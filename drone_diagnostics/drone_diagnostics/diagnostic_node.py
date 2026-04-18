#!/usr/bin/env python3
"""Real-time drone diagnostic node for PX4 — /fmu/out telemetry only."""

import math
import time
from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from px4_msgs.msg import (
    BatteryStatus,
    Cpuload,
    EscStatus,
    EstimatorStatus,
    EstimatorStatusFlags,
    FailsafeFlags,
    GeofenceResult,
    HomePosition,
    RcChannels,
    SensorCombined,
    SensorGps,
    TelemetryStatus,
    TimesyncStatus,
    VehicleAirData,
    VehicleAngularVelocity,
    VehicleAttitude,
    VehicleImuStatus,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)

# ---------------------------------------------------------------------------
# QoS profiles
# ---------------------------------------------------------------------------

# PX4 topics use BEST_EFFORT + TRANSIENT_LOCAL (actual uXRCE-DDS bridge QoS).
# TRANSIENT_LOCAL means the DDS middleware retains the last value for late-joining
# subscribers — critical for slow topics like vehicle_status (2 Hz).
QOS_PX4 = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

OK = DiagnosticStatus.OK
WARN = DiagnosticStatus.WARN
ERROR = DiagnosticStatus.ERROR

# nav_state readable names (PX4 v1.14)
NAV_STATE = {
    0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION",
    4: "AUTO_LOITER", 5: "AUTO_RTL", 6: "ACRO", 8: "DESCEND",
    9: "TERMINATION", 10: "OFFBOARD", 14: "OFFBOARD",
    15: "STAB", 17: "AUTO_TAKEOFF", 18: "AUTO_LAND",
    19: "AUTO_FOLLOW_TARGET", 20: "PRECLAND",
}
ARMING_STATE = {1: "DISARMED", 2: "ARMED"}

# EstimatorStatusFlags.fs_* fault fields — any True = numerical error in the filter
EKF_FAULT_FIELDS = [
    "fs_bad_mag_x", "fs_bad_mag_y", "fs_bad_mag_z",
    "fs_bad_hdg", "fs_bad_mag_decl",
    "fs_bad_airspeed", "fs_bad_sideslip",
    "fs_bad_optflow_x", "fs_bad_optflow_y",
    "fs_bad_acc_bias", "fs_bad_acc_vertical", "fs_bad_acc_clipping",
]

# EstimatorStatusFlags.reject_* fields — any True = sensor measurement rejected
EKF_REJECT_FIELDS = [
    "reject_hor_vel", "reject_ver_vel",
    "reject_hor_pos", "reject_ver_pos",
    "reject_yaw", "reject_airspeed", "reject_sideslip",
    "reject_hagl", "reject_optflow_x", "reject_optflow_y",
]

# VehicleStatus.failure_detector_status bit definitions (mask, name)
FAILURE_DETECTOR_BITS = [
    (1,   "FAILURE_ROLL"),
    (2,   "FAILURE_PITCH"),
    (4,   "FAILURE_ALT"),
    (8,   "FAILURE_EXT"),
    (16,  "FAILURE_ARM_ESC"),
    (32,  "FAILURE_BATTERY"),
    (64,  "FAILURE_IMBALANCED_PROP"),
    (128, "FAILURE_MOTOR"),
]

# BatteryStatus.warning enum names (note: 5 is unassigned)
BATTERY_WARNING_NAMES = {
    0: "NONE", 1: "LOW", 2: "CRITICAL", 3: "EMERGENCY",
    4: "FAILED", 6: "UNHEALTHY", 7: "CHARGING",
}

# BatteryStatus.faults bitmask names (indices 0–10)
BATTERY_FAULT_NAMES = [
    "DEEP_DISCHARGE", "SPIKES", "CELL_FAIL", "OVER_CURRENT",
    "OVER_TEMPERATURE", "UNDER_TEMPERATURE", "INCOMPATIBLE_VOLTAGE",
    "INCOMPATIBLE_FIRMWARE", "INCOMPATIBLE_MODEL", "HARDWARE_FAILURE",
    "OVER_TEMPERATURE_2",   # PX4 has two distinct over-temp bits; index 10 reuses the name
]

# BatteryStatus.warning values that map to ERROR level
BATTERY_ERROR_WARNINGS = frozenset({2, 3, 4, 6})  # CRITICAL, EMERGENCY, FAILED, UNHEALTHY

GPS_FIX_TYPE_NAMES = {
    0: "NO_FIX", 1: "NO_FIX", 2: "2D_FIX", 3: "3D_FIX",
    4: "DGPS", 5: "RTK_FLOAT", 6: "RTK_FIXED",
}

# EscReport.failures bitmask (bit index → name, value = 1 << index)
ESC_FAILURE_BITS = [
    (1 << 0, "OVER_CURRENT"),       (1 << 1, "OVER_VOLTAGE"),
    (1 << 2, "MOTOR_OVER_TEMP"),    (1 << 3, "OVER_RPM"),
    (1 << 4, "INCONSISTENT_CMD"),   (1 << 5, "MOTOR_STUCK"),
    (1 << 6, "GENERIC"),            (1 << 7, "MOTOR_WARN_TEMP"),
    (1 << 8, "WARN_ESC_TEMP"),      (1 << 9, "OVER_ESC_TEMP"),
]

GEOFENCE_ACTION_NAMES = {
    0: "NONE", 1: "WARN", 2: "HOLD", 3: "RTL", 4: "TERMINATE", 5: "LAND",
}

# FailsafeFlags fields monitored (any True → ERROR)
FAILSAFE_FIELDS = [
    "angular_velocity_invalid", "attitude_invalid",
    "local_altitude_invalid", "local_position_invalid",
    "local_position_invalid_relaxed", "local_velocity_invalid",
    "global_position_invalid", "offboard_control_signal_lost",
    "battery_warning", "battery_unhealthy",
    "geofence_breached", "local_position_accuracy_low",
    "mission_failure",
]


# ---------------------------------------------------------------------------
# TopicMonitor
# ---------------------------------------------------------------------------

class TopicMonitor:
    """Tracks message rate and staleness for a single topic."""

    def __init__(self, name: str, expected_hz: float, timeout_sec: float = 2.0) -> None:
        self.name = name
        self.expected_hz = expected_hz
        self.timeout_sec = timeout_sec
        self._timestamps: deque[float] = deque(maxlen=30)
        self._last_msg_time: Optional[float] = None

    def on_message(self) -> None:
        now = time.monotonic()
        self._timestamps.append(now)
        self._last_msg_time = now

    @property
    def is_alive(self) -> bool:
        if self._last_msg_time is None:
            return False
        return (time.monotonic() - self._last_msg_time) < self.timeout_sec

    @property
    def actual_hz(self) -> float:
        if len(self._timestamps) < 2:
            return 0.0
        window = list(self._timestamps)
        dt = window[-1] - window[0]
        return (len(window) - 1) / dt if dt > 0 else 0.0

    @property
    def staleness_sec(self) -> float:
        if self._last_msg_time is None:
            return float("inf")
        return time.monotonic() - self._last_msg_time

    def base_kvs(self) -> list[KeyValue]:
        stale = (
            f"{self.staleness_sec:.2f}s"
            if self.staleness_sec != float("inf")
            else "never received"
        )
        return [
            KeyValue(key="actual_hz", value=f"{self.actual_hz:.2f}"),
            KeyValue(key="expected_hz", value=f"{self.expected_hz:.1f}"),
            KeyValue(key="staleness", value=stale),
        ]

    def rate_status(self, min_ratio: float = 0.5):
        if not self.is_alive:
            msg = "No messages" if self._last_msg_time is None else f"Stale ({self.staleness_sec:.1f}s)"
            return ERROR, msg
        ratio = self.actual_hz / self.expected_hz if self.expected_hz > 0 else 1.0
        if ratio < min_ratio:
            return WARN, f"Rate low: {self.actual_hz:.1f}/{self.expected_hz:.0f} Hz"
        return OK, f"OK @ {self.actual_hz:.1f} Hz"


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class DroneDiagnosticNode(Node):
    def __init__(self) -> None:
        super().__init__("drone_diagnostic")

        self.declare_parameter("drone_namespace", "/pequi/hermit")
        self.declare_parameter("topic_timeout_sec", 2.0)
        self.declare_parameter("publish_rate_hz", 2.0)

        self.declare_parameter("cpu_warn_pct", 0.70)
        self.declare_parameter("cpu_error_pct", 0.90)
        self.declare_parameter("ram_warn_pct", 0.80)
        self.declare_parameter("timesync_warn_us", 500)
        self.declare_parameter("timesync_error_us", 2000)

        # Battery thresholds (4S LiPo defaults)
        self.declare_parameter("battery_warn_v", 14.4)
        self.declare_parameter("battery_error_v", 13.6)
        self.declare_parameter("battery_warn_pct", 0.25)
        self.declare_parameter("battery_error_pct", 0.10)

        # IMU vibration thresholds (m²/s⁴)
        self.declare_parameter("vibration_warn_m2s4", 4.0)
        self.declare_parameter("vibration_error_m2s4", 9.0)

        # EKF horizontal accuracy warn threshold (m)
        self.declare_parameter("ekf_horiz_accuracy_warn_m", 5.0)

        # Minimum rate ratio before a high-rate topic is WARN (fraction of expected Hz)
        self.declare_parameter("rate_min_ratio", 0.5)

        # IMU vibration — VehicleImuStatus primary path
        self.declare_parameter("imu_accel_vib_warn_ms2",  0.35)
        self.declare_parameter("imu_accel_vib_error_ms2", 0.70)
        self.declare_parameter("imu_gyro_vib_warn_rads",  0.07)
        self.declare_parameter("imu_gyro_vib_error_rads", 0.15)

        # Battery cell balance
        self.declare_parameter("battery_cell_delta_warn_v", 0.20)

        # ── Enable flags — existing checks ──────────────────────────────────
        self.declare_parameter("enable_vehicle_status",   True)
        self.declare_parameter("enable_failsafe",         True)
        self.declare_parameter("enable_land_detected",    True)
        self.declare_parameter("enable_cpuload",          True)
        self.declare_parameter("enable_timesync",         True)
        self.declare_parameter("enable_battery",          True)
        self.declare_parameter("enable_imu",              True)
        self.declare_parameter("enable_angular_velocity", True)
        self.declare_parameter("enable_local_position",   True)
        self.declare_parameter("enable_ekf_health",       True)
        self.declare_parameter("enable_imu_vibration",    True)
        self.declare_parameter("enable_ekf_status_flags", True)

        # ── Enable flags — new checks ────────────────────────────────────────
        self.declare_parameter("enable_gps",              True)
        self.declare_parameter("enable_rc_link",          True)
        self.declare_parameter("enable_attitude",         True)
        self.declare_parameter("enable_esc_health",       True)
        self.declare_parameter("enable_telemetry_link",   True)
        self.declare_parameter("enable_barometer",        True)
        self.declare_parameter("enable_home_position",    True)
        self.declare_parameter("enable_geofence",         True)

        # ── GPS thresholds ───────────────────────────────────────────────────
        self.declare_parameter("gps_fix_type_warn",    3)
        self.declare_parameter("gps_fix_type_error",   2)
        self.declare_parameter("gps_hdop_warn",        2.0)
        self.declare_parameter("gps_hdop_error",       5.0)
        self.declare_parameter("gps_satellites_warn",  6)
        self.declare_parameter("gps_satellites_error", 4)

        # ── RC Link thresholds ───────────────────────────────────────────────
        self.declare_parameter("rc_rssi_warn",  30)
        self.declare_parameter("rc_rssi_error", 10)

        # ── ESC Health thresholds ────────────────────────────────────────────
        self.declare_parameter("esc_temp_warn_c",  70.0)
        self.declare_parameter("esc_temp_error_c", 85.0)

        # ── Barometer plausibility bounds (Pa) ───────────────────────────────
        self.declare_parameter("baro_pressure_min_pa", 30000.0)
        self.declare_parameter("baro_pressure_max_pa", 110000.0)

        ns = self.get_parameter("drone_namespace").get_parameter_value().string_value
        timeout = self.get_parameter("topic_timeout_sec").get_parameter_value().double_value

        # ------------------------------------------------------------------ #
        # Topic monitors — fmu/out high-rate (name, expected_hz, timeout)    #
        # ------------------------------------------------------------------ #

        self._mon_imu = TopicMonitor(f"{ns}/fmu/out/sensor_combined", 100.0, timeout)
        self._mon_ang_vel = TopicMonitor(f"{ns}/fmu/out/vehicle_angular_velocity", 100.0, timeout)
        self._mon_local_pos = TopicMonitor(f"{ns}/fmu/out/vehicle_local_position", 100.0, timeout)
        self._mon_estimator = TopicMonitor(f"{ns}/fmu/out/estimator_status", 100.0, timeout)
        self._mon_estimator_flags = TopicMonitor(f"{ns}/fmu/out/estimator_status_flags", 100.0, timeout)
        self._mon_imu_status = TopicMonitor(f"{ns}/fmu/out/vehicle_imu_status", 100.0, timeout)
        self._mon_gps = TopicMonitor(f"{ns}/fmu/out/sensor_gps", 5.0, timeout * 3)

        # ------------------------------------------------------------------ #
        # Latest message state — fmu/out low-rate status topics              #
        # ------------------------------------------------------------------ #

        self._vehicle_status_msg: Optional[VehicleStatus] = None
        self._estimator_msg: Optional[EstimatorStatus] = None
        self._estimator_flags_msg: Optional[EstimatorStatusFlags] = None
        self._imu_status_msg: Optional[VehicleImuStatus] = None
        self._failsafe_msg: Optional[FailsafeFlags] = None
        self._cpuload_msg: Optional[Cpuload] = None
        self._timesync_msg: Optional[TimesyncStatus] = None
        self._land_detected_msg: Optional[VehicleLandDetected] = None
        self._local_pos_msg: Optional[VehicleLocalPosition] = None
        self._battery_msg: Optional[BatteryStatus] = None

        self._attitude_msg: Optional[VehicleAttitude] = None
        self._gps_msg: Optional[SensorGps] = None
        self._esc_status_msg: Optional[EscStatus] = None
        self._rc_channels_msg: Optional[RcChannels] = None
        self._telemetry_status_msg: Optional[TelemetryStatus] = None
        self._air_data_msg: Optional[VehicleAirData] = None
        self._geofence_result_msg: Optional[GeofenceResult] = None
        self._home_position_msg: Optional[HomePosition] = None
        self._prev_q_reset_counter: int = -1  # -1 = first message not yet seen

        # Accel samples for vibration analysis (last 30 at 100 Hz ≈ 0.3 s window)
        self._accel_samples: deque[tuple[float, float, float]] = deque(maxlen=30)

        # ------------------------------------------------------------------ #
        # Subscriptions — fmu/out only                                        #
        # ------------------------------------------------------------------ #

        # High-rate
        self.create_subscription(SensorCombined, f"{ns}/fmu/out/sensor_combined",
                                 self._cb_sensor_combined, QOS_PX4)
        self.create_subscription(VehicleAngularVelocity, f"{ns}/fmu/out/vehicle_angular_velocity",
                                 lambda _: self._mon_ang_vel.on_message(), QOS_PX4)
        self.create_subscription(VehicleLocalPosition, f"{ns}/fmu/out/vehicle_local_position",
                                 self._cb_local_pos, QOS_PX4)
        self.create_subscription(EstimatorStatus, f"{ns}/fmu/out/estimator_status",
                                 self._cb_estimator, QOS_PX4)

        # Low-rate status
        self.create_subscription(VehicleStatus, f"{ns}/fmu/out/vehicle_status",
                                 self._cb_vehicle_status, QOS_PX4)
        self.create_subscription(FailsafeFlags, f"{ns}/fmu/out/failsafe_flags",
                                 self._cb_failsafe, QOS_PX4)
        self.create_subscription(Cpuload, f"{ns}/fmu/out/cpuload",
                                 self._cb_cpuload, QOS_PX4)
        self.create_subscription(TimesyncStatus, f"{ns}/fmu/out/timesync_status",
                                 self._cb_timesync, QOS_PX4)
        self.create_subscription(VehicleLandDetected, f"{ns}/fmu/out/vehicle_land_detected",
                                 self._cb_land_detected, QOS_PX4)
        self.create_subscription(BatteryStatus, f"{ns}/fmu/out/battery_status",
                                 self._cb_battery, QOS_PX4)

        # EstimatorStatusFlags — authoritative per-boolean EKF state (100 Hz)
        self.create_subscription(EstimatorStatusFlags,
                                 f"{ns}/fmu/out/estimator_status_flags",
                                 self._cb_estimator_flags, QOS_PX4)

        # VehicleImuStatus — PX4-computed vibration metrics (100 Hz, optional)
        self.create_subscription(VehicleImuStatus,
                                 f"{ns}/fmu/out/vehicle_imu_status",
                                 self._cb_imu_status, QOS_PX4)

        # New low-rate status topics
        self.create_subscription(VehicleAttitude, f"{ns}/fmu/out/vehicle_attitude",
                                 self._cb_attitude, QOS_PX4)
        self.create_subscription(SensorGps, f"{ns}/fmu/out/sensor_gps",
                                 self._cb_gps, QOS_PX4)
        self.create_subscription(EscStatus, f"{ns}/fmu/out/esc_status",
                                 self._cb_esc_status, QOS_PX4)
        self.create_subscription(RcChannels, f"{ns}/fmu/out/rc_channels",
                                 self._cb_rc_channels, QOS_PX4)
        self.create_subscription(TelemetryStatus, f"{ns}/fmu/out/telemetry_status",
                                 self._cb_telemetry_status, QOS_PX4)
        self.create_subscription(VehicleAirData, f"{ns}/fmu/out/vehicle_air_data",
                                 self._cb_air_data, QOS_PX4)
        self.create_subscription(GeofenceResult, f"{ns}/fmu/out/geofence_result",
                                 self._cb_geofence_result, QOS_PX4)
        self.create_subscription(HomePosition, f"{ns}/fmu/out/home_position",
                                 self._cb_home_position, QOS_PX4)

        # ------------------------------------------------------------------ #
        # Publisher + timer                                                    #
        # ------------------------------------------------------------------ #

        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self.create_timer(1.0 / rate, self._publish_diagnostics)

        self.get_logger().info(f"Drone diagnostic node started (namespace={ns}, rate={rate:.0f}Hz)")

    # ------------------------------------------------------------------ #
    # Callbacks                                                            #
    # ------------------------------------------------------------------ #

    def _cb_sensor_combined(self, msg: SensorCombined) -> None:
        self._mon_imu.on_message()
        self._accel_samples.append((msg.accelerometer_m_s2[0],
                                    msg.accelerometer_m_s2[1],
                                    msg.accelerometer_m_s2[2]))

    def _cb_local_pos(self, msg: VehicleLocalPosition) -> None:
        self._mon_local_pos.on_message()
        self._local_pos_msg = msg

    def _cb_estimator(self, msg: EstimatorStatus) -> None:
        self._mon_estimator.on_message()
        self._estimator_msg = msg

    def _cb_vehicle_status(self, msg: VehicleStatus) -> None:
        self._vehicle_status_msg = msg

    def _cb_failsafe(self, msg: FailsafeFlags) -> None:
        self._failsafe_msg = msg

    def _cb_cpuload(self, msg: Cpuload) -> None:
        self._cpuload_msg = msg

    def _cb_timesync(self, msg: TimesyncStatus) -> None:
        self._timesync_msg = msg

    def _cb_land_detected(self, msg: VehicleLandDetected) -> None:
        self._land_detected_msg = msg

    def _cb_battery(self, msg: BatteryStatus) -> None:
        self._battery_msg = msg

    def _cb_estimator_flags(self, msg: EstimatorStatusFlags) -> None:
        self._mon_estimator_flags.on_message()
        self._estimator_flags_msg = msg

    def _cb_imu_status(self, msg: VehicleImuStatus) -> None:
        self._mon_imu_status.on_message()
        self._imu_status_msg = msg

    def _cb_attitude(self, msg: VehicleAttitude) -> None:
        self._attitude_msg = msg

    def _cb_gps(self, msg: SensorGps) -> None:
        self._mon_gps.on_message()
        self._gps_msg = msg

    def _cb_esc_status(self, msg: EscStatus) -> None:
        self._esc_status_msg = msg

    def _cb_rc_channels(self, msg: RcChannels) -> None:
        self._rc_channels_msg = msg

    def _cb_telemetry_status(self, msg: TelemetryStatus) -> None:
        self._telemetry_status_msg = msg

    def _cb_air_data(self, msg: VehicleAirData) -> None:
        self._air_data_msg = msg

    def _cb_geofence_result(self, msg: GeofenceResult) -> None:
        self._geofence_result_msg = msg

    def _cb_home_position(self, msg: HomePosition) -> None:
        self._home_position_msg = msg

    # ------------------------------------------------------------------ #
    # Diagnostic builders — fmu/out telemetry                             #
    # ------------------------------------------------------------------ #

    def _diag_vehicle_status(self) -> DiagnosticStatus:
        # HEALTH CRITERIA — PX4: Vehicle Status (/fmu/out/vehicle_status)
        # ─────────────────────────────────────────────────────────────────
        # OK    : failsafe=False, arming_state ∈ {DISARMED=1, ARMED=2},
        #         failure_detector_status==0
        # WARN  : arming_state outside {1, 2}  (unexpected/transitional state)
        #         OR gcs_connection_lost=True  (datalink down, no failsafe yet)
        #         OR pre_flight_checks_pass=False while ARMED
        # ERROR : failsafe=True
        #         OR any failure_detector_status bit set
        # NO MSG: ERROR
        # ─────────────────────────────────────────────────────────────────
        if self._vehicle_status_msg is None:
            return DiagnosticStatus(level=ERROR, name="PX4: Vehicle Status",
                                    message="No messages received", hardware_id="px4_fmu")
        s = self._vehicle_status_msg
        nav = NAV_STATE.get(s.nav_state, f"UNKNOWN({s.nav_state})")
        arm = ARMING_STATE.get(s.arming_state, f"UNKNOWN({s.arming_state})")

        level = OK
        msg = f"{arm} | {nav}"
        if s.failsafe:
            level = ERROR
            msg = f"FAILSAFE ACTIVE | {arm} | {nav}"
        elif s.arming_state not in (1, 2):
            level = WARN

        # Failure detector bitmask — identifies which failure triggered
        fd = s.failure_detector_status
        active_failures = [name for mask, name in FAILURE_DETECTOR_BITS if fd & mask]
        if active_failures and level != ERROR:
            level = ERROR
            msg = f"Failure detector: {','.join(active_failures)}"

        # GCS link loss — WARN (don't escalate past ERROR)
        if s.gcs_connection_lost and level == OK:
            level = WARN
            msg = f"GCS link lost (x{s.gcs_connection_lost_counter})"

        # Pre-flight checks failing while armed
        if not s.pre_flight_checks_pass and s.arming_state == 2 and level == OK:
            level = WARN
            msg = "Pre-flight checks failing while ARMED"

        kvs = [
            KeyValue(key="arming_state", value=arm),
            KeyValue(key="nav_state", value=nav),
            KeyValue(key="failsafe", value=str(s.failsafe)),
            KeyValue(key="failure_detector", value=hex(fd)),
            KeyValue(key="active_failures", value=",".join(active_failures) or "none"),
            KeyValue(key="pre_flight_checks_pass", value=str(s.pre_flight_checks_pass)),
            KeyValue(key="gcs_connection_lost", value=str(s.gcs_connection_lost)),
            KeyValue(key="gcs_lost_count", value=str(s.gcs_connection_lost_counter)),
        ]
        return DiagnosticStatus(level=level, name="PX4: Vehicle Status",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_failsafe(self) -> DiagnosticStatus:
        # HEALTH CRITERIA — PX4: Failsafe Flags (/fmu/out/failsafe_flags)
        # ─────────────────────────────────────────────────────────────────
        # OK    : all monitored flags are False
        # ERROR : any of the 13 monitored flags is True
        # NO MSG: WARN
        #
        # Monitored flags (any True → ERROR):
        #   angular_velocity_invalid, attitude_invalid,
        #   local_altitude_invalid, local_position_invalid,
        #   local_position_invalid_relaxed, local_velocity_invalid,
        #   global_position_invalid, offboard_control_signal_lost,
        #   battery_warning, battery_unhealthy,
        #   geofence_breached, local_position_accuracy_low, mission_failure
        # ─────────────────────────────────────────────────────────────────
        if self._failsafe_msg is None:
            return DiagnosticStatus(level=WARN, name="PX4: Failsafe Flags",
                                    message="No messages received", hardware_id="px4_fmu")
        f = self._failsafe_msg
        active = [field for field in FAILSAFE_FIELDS if getattr(f, field)]
        level = ERROR if active else OK
        msg = f"Active: {','.join(active)}" if active else "No active failsafes"
        kvs = [KeyValue(key=field, value="TRUE") for field in active]
        return DiagnosticStatus(level=level, name="PX4: Failsafe Flags",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_land_detected(self) -> DiagnosticStatus:
        # HEALTH CRITERIA — PX4: Land Detected (/fmu/out/vehicle_land_detected)
        # ─────────────────────────────────────────────────────────────────
        # OK    : freefall=False  (state may be LANDED / MAYBE_LANDED / IN-AIR)
        # WARN  : freefall=True
        # NO MSG: WARN
        # ─────────────────────────────────────────────────────────────────
        if self._land_detected_msg is None:
            return DiagnosticStatus(level=WARN, name="PX4: Land Detected",
                                    message="No messages received", hardware_id="px4_fmu")
        ld = self._land_detected_msg
        state = "LANDED" if ld.landed else ("MAYBE_LANDED" if ld.maybe_landed else "IN-AIR")
        level = WARN if ld.freefall else OK
        msg = "FREEFALL DETECTED" if ld.freefall else state
        kvs = [
            KeyValue(key="state", value=state),
            KeyValue(key="landed", value=str(ld.landed)),
            KeyValue(key="freefall", value=str(ld.freefall)),
            KeyValue(key="maybe_landed", value=str(ld.maybe_landed)),
            KeyValue(key="ground_contact", value=str(ld.ground_contact)),
        ]
        return DiagnosticStatus(level=level, name="PX4: Land Detected",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_cpuload(self) -> DiagnosticStatus:
        # HEALTH CRITERIA — PX4: CPU Load (/fmu/out/cpuload)
        # ─────────────────────────────────────────────────────────────────
        # OK    : cpu ≤ cpu_warn_pct (0.70) AND ram ≤ ram_warn_pct (0.80)
        # WARN  : cpu > cpu_warn_pct (0.70)
        # ERROR : cpu > cpu_error_pct (0.90)  OR  ram > ram_warn_pct (0.80)
        # NO MSG: WARN
        # ─────────────────────────────────────────────────────────────────
        cpu_warn = self.get_parameter("cpu_warn_pct").get_parameter_value().double_value
        cpu_error = self.get_parameter("cpu_error_pct").get_parameter_value().double_value
        ram_warn = self.get_parameter("ram_warn_pct").get_parameter_value().double_value
        if self._cpuload_msg is None:
            return DiagnosticStatus(level=WARN, name="PX4: CPU Load",
                                    message="No messages received", hardware_id="px4_fmu")
        c = self._cpuload_msg
        cpu_pct = c.load * 100.0
        ram_pct = c.ram_usage * 100.0
        if c.load > cpu_error or c.ram_usage > ram_warn:
            level, msg = ERROR, f"CPU {cpu_pct:.0f}% RAM {ram_pct:.0f}%"
        elif c.load > cpu_warn:
            level, msg = WARN, f"CPU high: {cpu_pct:.0f}%"
        else:
            level, msg = OK, f"CPU {cpu_pct:.0f}% RAM {ram_pct:.0f}%"
        kvs = [
            KeyValue(key="cpu_percent", value=f"{cpu_pct:.1f}"),
            KeyValue(key="ram_percent", value=f"{ram_pct:.1f}"),
        ]
        return DiagnosticStatus(level=level, name="PX4: CPU Load",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_timesync(self) -> DiagnosticStatus:
        # HEALTH CRITERIA — PX4: Time Sync (/fmu/out/timesync_status)
        # ─────────────────────────────────────────────────────────────────
        # OK    : |estimated_offset| ≤ timesync_warn_us (default 500 µs)
        # WARN  : |estimated_offset| > timesync_warn_us (500 µs)
        # ERROR : |estimated_offset| > timesync_error_us (2000 µs)
        # NO MSG: WARN
        # ─────────────────────────────────────────────────────────────────
        ts_warn = self.get_parameter("timesync_warn_us").get_parameter_value().integer_value
        ts_error = self.get_parameter("timesync_error_us").get_parameter_value().integer_value
        if self._timesync_msg is None:
            return DiagnosticStatus(level=WARN, name="PX4: Time Sync",
                                    message="No messages received", hardware_id="px4_fmu")
        t = self._timesync_msg
        offset_us = abs(t.estimated_offset) / 1000.0
        if offset_us > ts_error:
            level, msg = ERROR, f"Large offset: {offset_us:.0f}µs"
        elif offset_us > ts_warn:
            level, msg = WARN, f"Offset elevated: {offset_us:.0f}µs"
        else:
            level, msg = OK, f"Synced ({offset_us:.0f}µs)"
        kvs = [
            KeyValue(key="offset_us", value=f"{offset_us:.1f}"),
            KeyValue(key="round_trip_time_us", value=f"{t.round_trip_time / 1000.0:.1f}"),
            KeyValue(key="estimated_offset_ns", value=str(t.estimated_offset)),
        ]
        return DiagnosticStatus(level=level, name="PX4: Time Sync",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_battery(self) -> DiagnosticStatus:
        # HEALTH CRITERIA — PX4: Battery Health (/fmu/out/battery_status)
        # 4S LiPo reference: full ≈ 16.8 V, nominal ≈ 14.8 V
        # ─────────────────────────────────────────────────────────────────
        # Level determined by PX4's own battery state machine first, then
        # voltage/remaining thresholds as a secondary guard.
        #
        # OK    : warning==NONE AND voltage ≥ battery_warn_v AND remaining ≥ battery_warn_pct
        # WARN  : warning==LOW OR warning==CHARGING
        #         OR voltage < battery_warn_v  OR  remaining < battery_warn_pct
        #         OR max_cell_voltage_delta > battery_cell_delta_warn_v (0.20 V)
        # ERROR : warning ∈ {CRITICAL, EMERGENCY, FAILED, UNHEALTHY}
        #         OR voltage < battery_error_v  OR  remaining < battery_error_pct
        #         OR any faults bitmask bit set
        # NO MSG: WARN
        # ─────────────────────────────────────────────────────────────────
        warn_v = self.get_parameter("battery_warn_v").get_parameter_value().double_value
        error_v = self.get_parameter("battery_error_v").get_parameter_value().double_value
        warn_pct = self.get_parameter("battery_warn_pct").get_parameter_value().double_value
        error_pct = self.get_parameter("battery_error_pct").get_parameter_value().double_value
        cell_delta_warn = self.get_parameter("battery_cell_delta_warn_v").get_parameter_value().double_value
        if self._battery_msg is None:
            return DiagnosticStatus(level=WARN, name="PX4: Battery Health",
                                    message="No messages received", hardware_id="px4_fmu")
        b = self._battery_msg
        voltage = b.voltage_v
        remaining = b.remaining  # 0.0–1.0
        remaining_pct = remaining * 100.0

        # PX4 battery state machine takes precedence
        px4_warn = b.warning
        warning_name = BATTERY_WARNING_NAMES.get(px4_warn, f"UNKNOWN({px4_warn})")
        if px4_warn in BATTERY_ERROR_WARNINGS:
            level = ERROR
            msg = f"Battery {warning_name}: {voltage:.2f}V {remaining_pct:.0f}%"
        elif px4_warn == 1:               # LOW
            level = WARN
            msg = f"Battery LOW: {voltage:.2f}V {remaining_pct:.0f}%"
        elif voltage < error_v or remaining < error_pct:
            level = ERROR
            msg = f"CRITICAL: {voltage:.2f}V {remaining_pct:.0f}%"
        elif voltage < warn_v or remaining < warn_pct:
            level = WARN
            msg = f"Low battery: {voltage:.2f}V {remaining_pct:.0f}%"
        else:
            level = OK
            msg = f"{voltage:.2f}V {remaining_pct:.0f}% ({b.current_a:.1f}A)"

        # Fault bitmask — smart battery reported faults
        active_faults = [BATTERY_FAULT_NAMES[i] for i in range(len(BATTERY_FAULT_NAMES)) if (b.faults >> i) & 1]
        if active_faults and level != ERROR:
            level = ERROR
            msg = f"Battery faults: {','.join(active_faults)}"

        # Cell imbalance
        cell_delta = b.max_cell_voltage_delta
        if not math.isnan(cell_delta) and cell_delta > cell_delta_warn and level == OK:
            level = WARN
            msg = f"Cell imbalance: \u0394{cell_delta:.3f}V"

        kvs = [
            KeyValue(key="voltage_v", value=f"{voltage:.3f}"),
            KeyValue(key="remaining_pct", value=f"{remaining_pct:.1f}"),
            KeyValue(key="current_a", value=f"{b.current_a:.2f}"),
            KeyValue(key="discharged_mah", value=f"{b.discharged_mah:.0f}"),
            KeyValue(key="px4_warning", value=warning_name),
            KeyValue(key="faults", value=",".join(active_faults) or "none"),
            KeyValue(key="cell_delta_v", value="NaN" if math.isnan(cell_delta) else f"{cell_delta:.3f}"),
            KeyValue(key="temperature_c", value="NaN" if math.isnan(b.temperature) else f"{b.temperature:.1f}"),
            KeyValue(key="time_remaining_s", value="NaN" if math.isnan(b.time_remaining_s) else f"{b.time_remaining_s:.0f}"),
        ]
        return DiagnosticStatus(level=level, name="PX4: Battery Health",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_imu(self) -> DiagnosticStatus:
        # HEALTH CRITERIA — PX4: IMU (/fmu/out/sensor_combined)
        # ─────────────────────────────────────────────────────────────────
        # OK    : topic alive AND rate ≥ rate_min_ratio * 100 Hz
        # WARN  : topic alive AND rate < rate_min_ratio * 100 Hz
        # ERROR : topic stale (> topic_timeout_sec) or never received
        # ─────────────────────────────────────────────────────────────────
        min_ratio = self.get_parameter("rate_min_ratio").get_parameter_value().double_value
        level, msg = self._mon_imu.rate_status(min_ratio)
        return DiagnosticStatus(level=level, name="PX4: IMU (sensor_combined)",
                                message=msg, hardware_id="px4_fmu", values=self._mon_imu.base_kvs())

    def _diag_ang_vel(self) -> DiagnosticStatus:
        # HEALTH CRITERIA — PX4: Angular Velocity (/fmu/out/vehicle_angular_velocity)
        # ─────────────────────────────────────────────────────────────────
        # OK    : topic alive AND rate ≥ rate_min_ratio * 100 Hz
        # WARN  : topic alive AND rate < rate_min_ratio * 100 Hz
        # ERROR : topic stale (> topic_timeout_sec) or never received
        # ─────────────────────────────────────────────────────────────────
        min_ratio = self.get_parameter("rate_min_ratio").get_parameter_value().double_value
        level, msg = self._mon_ang_vel.rate_status(min_ratio)
        return DiagnosticStatus(level=level, name="PX4: Angular Velocity",
                                message=msg, hardware_id="px4_fmu", values=self._mon_ang_vel.base_kvs())

    def _diag_local_position(self) -> DiagnosticStatus:
        # HEALTH CRITERIA — PX4: Local Position (/fmu/out/vehicle_local_position)
        # ─────────────────────────────────────────────────────────────────
        # OK    : topic alive AND xy_valid=True AND z_valid=True
        # WARN  : topic alive AND z_valid=False  (altitude unreliable)
        # ERROR : topic stale/absent  OR  xy_valid=False  (no horizontal fix)
        # ─────────────────────────────────────────────────────────────────
        min_ratio = self.get_parameter("rate_min_ratio").get_parameter_value().double_value
        level, msg = self._mon_local_pos.rate_status(min_ratio)
        kvs = self._mon_local_pos.base_kvs()
        if self._local_pos_msg is not None and self._mon_local_pos.is_alive:
            p = self._local_pos_msg
            kvs += [
                KeyValue(key="xy_valid", value=str(p.xy_valid)),
                KeyValue(key="z_valid", value=str(p.z_valid)),
                KeyValue(key="xy_global", value=str(p.xy_global)),
                KeyValue(key="eph_m", value=f"{p.eph:.3f}"),
                KeyValue(key="epv_m", value=f"{p.epv:.3f}"),
            ]
            if not p.xy_valid and level == OK:
                level, msg = ERROR, "XY position invalid"
            elif not p.z_valid and level == OK:
                level, msg = WARN, "Z position invalid"
        return DiagnosticStatus(level=level, name="PX4: Local Position",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_ekf_health(self) -> DiagnosticStatus:
        # HEALTH CRITERIA — PX4: EKF Health (/fmu/out/estimator_status)
        # ─────────────────────────────────────────────────────────────────
        # OK    : topic alive AND pos_horiz_accuracy ≤ ekf_horiz_accuracy_warn_m
        # WARN  : topic alive AND pos_horiz_accuracy > ekf_horiz_accuracy_warn_m
        # ERROR : topic stale or never received
        #
        # Innovation fault details are reported by PX4: EKF Status Flags
        # (estimator_status_flags) using the authoritative per-boolean fields.
        # ─────────────────────────────────────────────────────────────────
        min_ratio = self.get_parameter("rate_min_ratio").get_parameter_value().double_value
        level, msg = self._mon_estimator.rate_status(min_ratio)
        kvs = self._mon_estimator.base_kvs()
        if self._estimator_msg is not None and self._mon_estimator.is_alive:
            e = self._estimator_msg
            kvs += [
                KeyValue(key="pos_horiz_accuracy_m", value=f"{e.pos_horiz_accuracy:.3f}"),
                KeyValue(key="pos_vert_accuracy_m", value=f"{e.pos_vert_accuracy:.3f}"),
            ]
            ekf_acc_warn = self.get_parameter("ekf_horiz_accuracy_warn_m").get_parameter_value().double_value
            if e.pos_horiz_accuracy > ekf_acc_warn and level == OK:
                level = WARN
                msg = f"Low horizontal accuracy ({e.pos_horiz_accuracy:.1f}m)"
        return DiagnosticStatus(level=level, name="PX4: EKF Health",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_imu_vibration(self) -> DiagnosticStatus:
        # HEALTH CRITERIA — PX4: IMU Vibration
        # ─────────────────────────────────────────────────────────────────
        # PRIMARY path (vehicle_imu_status alive):
        #   Uses PX4-computed accel_vibration_metric (m/s²) and
        #   gyro_vibration_metric (rad/s). Also reports clipping counts,
        #   error counts, and temperatures.
        #   OK    : accel ≤ imu_accel_vib_warn_ms2 AND gyro ≤ imu_gyro_vib_warn_rads
        #   WARN  : either metric exceeds warn threshold
        #   ERROR : either metric exceeds error threshold
        #
        # FALLBACK path (vehicle_imu_status absent):
        #   Variance of last 30 sensor_combined accel samples (existing logic).
        #   OK    : max variance ≤ vibration_warn_m2s4
        #   WARN  : max variance > vibration_warn_m2s4
        #   ERROR : max variance > vibration_error_m2s4
        # ─────────────────────────────────────────────────────────────────
        if self._mon_imu_status.is_alive and self._imu_status_msg is not None:
            # PRIMARY PATH — PX4-native metrics
            a_warn  = self.get_parameter("imu_accel_vib_warn_ms2").get_parameter_value().double_value
            a_error = self.get_parameter("imu_accel_vib_error_ms2").get_parameter_value().double_value
            g_warn  = self.get_parameter("imu_gyro_vib_warn_rads").get_parameter_value().double_value
            g_error = self.get_parameter("imu_gyro_vib_error_rads").get_parameter_value().double_value
            s = self._imu_status_msg
            a = s.accel_vibration_metric
            g = s.gyro_vibration_metric
            if a > a_error or g > g_error:
                level = ERROR
                msg = f"High vib: accel={a:.3f}m/s² gyro={g:.3f}rad/s"
            elif a > a_warn or g > g_warn:
                level = WARN
                msg = f"Elevated vib: accel={a:.3f}m/s² gyro={g:.3f}rad/s"
            else:
                level = OK
                msg = f"OK accel={a:.3f}m/s² gyro={g:.3f}rad/s"
            kvs = [
                KeyValue(key="accel_vibration_metric_ms2", value=f"{a:.4f}"),
                KeyValue(key="gyro_vibration_metric_rads", value=f"{g:.4f}"),
                KeyValue(key="delta_angle_coning_rad2", value=f"{s.delta_angle_coning_metric:.6f}"),
                KeyValue(key="accel_clipping_x", value=str(s.accel_clipping[0])),
                KeyValue(key="accel_clipping_y", value=str(s.accel_clipping[1])),
                KeyValue(key="accel_clipping_z", value=str(s.accel_clipping[2])),
                KeyValue(key="gyro_clipping_x",  value=str(s.gyro_clipping[0])),
                KeyValue(key="gyro_clipping_y",  value=str(s.gyro_clipping[1])),
                KeyValue(key="gyro_clipping_z",  value=str(s.gyro_clipping[2])),
                KeyValue(key="accel_error_count", value=str(s.accel_error_count)),
                KeyValue(key="gyro_error_count",  value=str(s.gyro_error_count)),
                KeyValue(key="temperature_accel_c", value=f"{s.temperature_accel:.1f}"),
                KeyValue(key="temperature_gyro_c",  value=f"{s.temperature_gyro:.1f}"),
                KeyValue(key="accel_rate_hz", value=f"{s.accel_rate_hz:.1f}"),
                KeyValue(key="source", value="vehicle_imu_status"),
            ]
            return DiagnosticStatus(level=level, name="PX4: IMU Vibration",
                                    message=msg, hardware_id="px4_fmu", values=kvs)

        # FALLBACK PATH — accel variance from sensor_combined samples
        vib_warn = self.get_parameter("vibration_warn_m2s4").get_parameter_value().double_value
        vib_error = self.get_parameter("vibration_error_m2s4").get_parameter_value().double_value
        if len(self._accel_samples) < 10:
            return DiagnosticStatus(
                level=WARN, name="PX4: IMU Vibration",
                message="Insufficient samples", hardware_id="px4_fmu",
                values=[
                    KeyValue(key="sample_count", value=str(len(self._accel_samples))),
                    KeyValue(key="source", value="variance_fallback"),
                ]
            )
        n = len(self._accel_samples)
        ax, ay, az = zip(*self._accel_samples)

        def variance(vals: tuple[float, ...]) -> float:
            mean = sum(vals) / len(vals)
            return sum((v - mean) ** 2 for v in vals) / len(vals)

        vx, vy, vz = variance(ax), variance(ay), variance(az)
        max_var = max(vx, vy, vz)

        if max_var > vib_error:
            level = ERROR
            msg = f"High vibration: {max_var:.2f} m²/s⁴"
        elif max_var > vib_warn:
            level = WARN
            msg = f"Elevated vibration: {max_var:.2f} m²/s⁴"
        else:
            level = OK
            msg = f"OK ({max_var:.2f} m²/s⁴)"

        kvs = [
            KeyValue(key="accel_var_x",   value=f"{vx:.4f}"),
            KeyValue(key="accel_var_y",   value=f"{vy:.4f}"),
            KeyValue(key="accel_var_z",   value=f"{vz:.4f}"),
            KeyValue(key="accel_var_max", value=f"{max_var:.4f}"),
            KeyValue(key="sample_count",  value=str(n)),
            KeyValue(key="source",        value="variance_fallback"),
        ]
        return DiagnosticStatus(level=level, name="PX4: IMU Vibration",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_ekf_status_flags(self) -> DiagnosticStatus:
        # HEALTH CRITERIA — PX4: EKF Status Flags (/fmu/out/estimator_status_flags)
        # ─────────────────────────────────────────────────────────────────
        # Authoritative per-boolean EKF state from PX4. Unlike EstimatorStatus,
        # field layout is stable across PX4 minor versions.
        #
        # OK    : all fs_* False  AND  all reject_* False
        #         AND cs_tilt_align=True AND cs_yaw_align=True
        #         AND cs_inertial_dead_reckoning=False
        # WARN  : any reject_* True  (measurement rejected, filter degraded)
        #         OR cs_tilt_align=False OR cs_yaw_align=False (not yet aligned)
        #         OR cs_inertial_dead_reckoning=True (no external position constraint)
        # ERROR : any fs_* True  (numerical error in the filter math)
        # NO MSG: WARN
        # ─────────────────────────────────────────────────────────────────
        if self._estimator_flags_msg is None:
            return DiagnosticStatus(level=WARN, name="PX4: EKF Status Flags",
                                    message="No messages received", hardware_id="px4_fmu")
        f = self._estimator_flags_msg

        faults  = [field for field in EKF_FAULT_FIELDS  if getattr(f, field)]
        rejects = [field for field in EKF_REJECT_FIELDS if getattr(f, field)]

        warn_conditions = (
            (["tilt_not_aligned"]        if not f.cs_tilt_align             else []) +
            (["yaw_not_aligned"]         if not f.cs_yaw_align              else []) +
            (["inertial_dead_reckoning"] if f.cs_inertial_dead_reckoning    else [])
        )

        if faults:
            level = ERROR
            msg = f"EKF faults: {','.join(faults)}"
        elif rejects:
            level = WARN
            msg = f"Innovation rejected: {','.join(rejects)}"
        elif warn_conditions:
            level = WARN
            msg = f"EKF warnings: {','.join(warn_conditions)}"
        else:
            gps_str  = "fusing" if f.cs_gps      else "off"
            baro_str = "fusing" if f.cs_baro_hgt  else "off"
            level = OK
            msg = f"OK (GPS={gps_str}, baro={baro_str})"

        kvs = [
            KeyValue(key="cs_tilt_align",             value=str(f.cs_tilt_align)),
            KeyValue(key="cs_yaw_align",              value=str(f.cs_yaw_align)),
            KeyValue(key="cs_gps",                    value=str(f.cs_gps)),
            KeyValue(key="cs_baro_hgt",               value=str(f.cs_baro_hgt)),
            KeyValue(key="cs_mag",                    value=str(f.cs_mag)),
            KeyValue(key="cs_inertial_dead_reckoning",value=str(f.cs_inertial_dead_reckoning)),
            KeyValue(key="cs_wind_dead_reckoning",    value=str(f.cs_wind_dead_reckoning)),
            KeyValue(key="cs_mag_fault",              value=str(f.cs_mag_fault)),
            KeyValue(key="active_faults",             value=",".join(faults)  or "none"),
            KeyValue(key="active_rejects",            value=",".join(rejects) or "none"),
            KeyValue(key="fault_status_changes",      value=str(f.fault_status_changes)),
            KeyValue(key="innovation_fault_status_changes", value=str(f.innovation_fault_status_changes)),
        ]
        return DiagnosticStatus(level=level, name="PX4: EKF Status Flags",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_gps(self) -> DiagnosticStatus:
        # HEALTH CRITERIA — PX4: GPS Quality (/fmu/out/sensor_gps)
        # ─────────────────────────────────────────────────────────────────
        # OK    : fix_type >= gps_fix_type_warn(3) AND hdop <= gps_hdop_warn(2.0)
        #         AND satellites_used >= gps_satellites_warn(6) AND jamming_indicator==0
        # WARN  : any warn threshold exceeded OR jamming_indicator > 0
        # ERROR : any error threshold exceeded OR topic stale
        # NO MSG: WARN
        # ─────────────────────────────────────────────────────────────────
        if self._gps_msg is None:
            return DiagnosticStatus(level=WARN, name="PX4: GPS Quality",
                                    message="No messages received", hardware_id="px4_fmu")
        g = self._gps_msg
        fix_warn  = self.get_parameter("gps_fix_type_warn").get_parameter_value().integer_value
        fix_err   = self.get_parameter("gps_fix_type_error").get_parameter_value().integer_value
        hdop_warn = self.get_parameter("gps_hdop_warn").get_parameter_value().double_value
        hdop_err  = self.get_parameter("gps_hdop_error").get_parameter_value().double_value
        sat_warn  = self.get_parameter("gps_satellites_warn").get_parameter_value().integer_value
        sat_err   = self.get_parameter("gps_satellites_error").get_parameter_value().integer_value

        level = OK
        problems: list[str] = []

        if not self._mon_gps.is_alive:
            stale = f"{self._mon_gps.staleness_sec:.1f}s"
            return DiagnosticStatus(level=ERROR, name="PX4: GPS Quality",
                                    message=f"Stale ({stale})", hardware_id="px4_fmu",
                                    values=self._mon_gps.base_kvs())

        if g.fix_type < fix_err or g.hdop > hdop_err or g.satellites_used < sat_err:
            level = ERROR
        elif g.fix_type < fix_warn or g.hdop > hdop_warn or g.satellites_used < sat_warn:
            level = WARN

        if g.fix_type < fix_warn:
            problems.append(f"fix={GPS_FIX_TYPE_NAMES.get(g.fix_type, str(g.fix_type))}")
        if g.satellites_used < sat_warn:
            problems.append(f"sats={g.satellites_used}")
        if g.hdop > hdop_warn:
            problems.append(f"hdop={g.hdop:.1f}")
        if g.jamming_indicator > 0:
            if level == OK:
                level = WARN
            problems.append(f"jamming={g.jamming_indicator}")

        msg = f"OK | {GPS_FIX_TYPE_NAMES.get(g.fix_type, str(g.fix_type))} sats={g.satellites_used} hdop={g.hdop:.1f}" \
              if not problems else ", ".join(problems)

        kvs = [
            KeyValue(key="fix_type",          value=GPS_FIX_TYPE_NAMES.get(g.fix_type, str(g.fix_type))),
            KeyValue(key="satellites_used",   value=str(g.satellites_used)),
            KeyValue(key="hdop",              value=f"{g.hdop:.2f}"),
            KeyValue(key="vdop",              value=f"{g.vdop:.2f}"),
            KeyValue(key="noise_per_ms",      value=str(g.noise_per_ms)),
            KeyValue(key="jamming_indicator", value=str(g.jamming_indicator)),
            KeyValue(key="actual_hz",         value=f"{self._mon_gps.actual_hz:.2f}"),
        ]
        return DiagnosticStatus(level=level, name="PX4: GPS Quality",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_rc_link(self) -> DiagnosticStatus:
        # HEALTH CRITERIA — PX4: RC Link (/fmu/out/rc_channels)
        # ─────────────────────────────────────────────────────────────────
        # OK    : signal_lost=False AND (rssi==0 OR rssi >= rc_rssi_warn)
        # WARN  : rssi > 0 AND rssi < rc_rssi_warn (degraded signal)
        # ERROR : signal_lost=True OR (rssi > 0 AND rssi < rc_rssi_error)
        # NO MSG: WARN
        # Note: rssi=0 means RSSI not wired/reported — not a failure
        #       signal_lost is the authoritative link-loss indicator
        # ─────────────────────────────────────────────────────────────────
        if self._rc_channels_msg is None:
            return DiagnosticStatus(level=WARN, name="PX4: RC Link",
                                    message="No messages received", hardware_id="px4_fmu")
        r = self._rc_channels_msg
        rssi_warn = self.get_parameter("rc_rssi_warn").get_parameter_value().integer_value
        rssi_err  = self.get_parameter("rc_rssi_error").get_parameter_value().integer_value

        if r.signal_lost:
            level = ERROR
            msg = "SIGNAL LOST"
        elif r.rssi > 0 and r.rssi < rssi_err:
            level = ERROR
            msg = f"RSSI critical: {r.rssi}"
        elif r.rssi > 0 and r.rssi < rssi_warn:
            level = WARN
            msg = f"RSSI low: {r.rssi}"
        else:
            level = OK
            rssi_str = str(r.rssi) if r.rssi > 0 else "N/A"
            msg = f"OK | channels={r.channel_count} rssi={rssi_str}"

        kvs = [
            KeyValue(key="signal_lost",      value=str(r.signal_lost)),
            KeyValue(key="rssi",             value=str(r.rssi)),
            KeyValue(key="channel_count",    value=str(r.channel_count)),
            KeyValue(key="frame_drop_count", value=str(r.frame_drop_count)),
        ]
        return DiagnosticStatus(level=level, name="PX4: RC Link",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_attitude(self) -> DiagnosticStatus:
        # HEALTH CRITERIA — PX4: Attitude (/fmu/out/vehicle_attitude)
        # ─────────────────────────────────────────────────────────────────
        # OK    : q_reset_counter unchanged AND |quaternion_norm - 1.0| < 0.01
        # WARN  : q_reset_counter has incremented since last check
        # ERROR : |quaternion_norm - 1.0| >= 0.05 (filter failure)
        # NO MSG: WARN
        # Note: q_reset_counter is uint8 — wrap-around detected
        # ─────────────────────────────────────────────────────────────────
        if self._attitude_msg is None:
            return DiagnosticStatus(level=WARN, name="PX4: Attitude",
                                    message="No messages received", hardware_id="px4_fmu")
        a = self._attitude_msg
        q = a.q
        q_norm = math.sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)
        norm_err = abs(q_norm - 1.0)

        current_reset = int(a.quat_reset_counter)
        if self._prev_q_reset_counter == -1:
            reset_delta = 0
        else:
            reset_delta = (current_reset - self._prev_q_reset_counter) % 256
        self._prev_q_reset_counter = current_reset

        if norm_err >= 0.05:
            level = ERROR
            msg = f"Invalid quaternion norm={q_norm:.4f}"
        elif reset_delta > 0:
            level = WARN
            msg = f"Attitude reset (x{reset_delta}) norm={q_norm:.4f}"
        else:
            level = OK
            msg = f"OK norm={q_norm:.4f} resets={current_reset}"

        kvs = [
            KeyValue(key="q_norm",           value=f"{q_norm:.4f}"),
            KeyValue(key="q_reset_counter",  value=str(current_reset)),
            KeyValue(key="reset_delta",      value=str(reset_delta)),
        ]
        return DiagnosticStatus(level=level, name="PX4: Attitude",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_esc_health(self) -> DiagnosticStatus:
        # HEALTH CRITERIA — PX4: ESC Health (/fmu/out/esc_status)
        # ─────────────────────────────────────────────────────────────────
        # OK    : all active ESC failures==0 AND max_temp <= esc_temp_warn_c
        # WARN  : any active ESC temp > esc_temp_warn_c
        # ERROR : any active ESC failures != 0 OR any temp > esc_temp_error_c
        # NO MSG: WARN
        # Active ESC = esc_rpm != 0 (ignore zero-RPM slots on a quad)
        # failures bitmask: bit0=OVER_CURRENT,1=OVER_VOLTAGE,2=MOTOR_OVER_TEMP,3=OVER_RPM
        # ─────────────────────────────────────────────────────────────────
        if self._esc_status_msg is None:
            return DiagnosticStatus(level=WARN, name="PX4: ESC Health",
                                    message="No messages received", hardware_id="px4_fmu")
        e = self._esc_status_msg
        temp_warn = self.get_parameter("esc_temp_warn_c").get_parameter_value().double_value
        temp_err  = self.get_parameter("esc_temp_error_c").get_parameter_value().double_value

        level = OK
        problems: list[str] = []
        kvs: list[KeyValue] = [
            KeyValue(key="esc_count",        value=str(e.esc_count)),
            KeyValue(key="esc_online_flags", value=bin(e.esc_online_flags)),
        ]

        count = min(e.esc_count, 8)
        esc_array = list(e.esc)
        for i in range(count):
            esc = esc_array[i]
            online = bool(e.esc_online_flags & (1 << i))
            if not online:
                continue
            temp = esc.esc_temperature
            failures = esc.failures
            fault_names = [name for mask, name in ESC_FAILURE_BITS if failures & mask]

            kvs += [
                KeyValue(key=f"esc{i}_rpm",      value=str(esc.esc_rpm)),
                KeyValue(key=f"esc{i}_temp_c",   value=f"{temp:.1f}"),
                KeyValue(key=f"esc{i}_failures",  value=",".join(fault_names) or "none"),
            ]
            if fault_names:
                level = ERROR
                problems.append(f"ESC{i}:{','.join(fault_names)}")
            elif temp > temp_err:
                level = ERROR
                problems.append(f"ESC{i}_OVERTEMP({temp:.0f}°C)")
            elif temp > temp_warn and level != ERROR:
                level = WARN
                problems.append(f"ESC{i}_HOT({temp:.0f}°C)")

        msg = ", ".join(problems) if problems else f"OK | {count} ESCs"
        return DiagnosticStatus(level=level, name="PX4: ESC Health",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_telemetry_link(self) -> DiagnosticStatus:
        # HEALTH CRITERIA — PX4: Telemetry Link (/fmu/out/telemetry_status)
        # ─────────────────────────────────────────────────────────────────
        # OK    : heartbeat_type_gcs=True
        # ERROR : heartbeat_type_gcs=False (GCS not sending MAVLink heartbeats)
        # NO MSG: WARN
        # Note: TelemetryStatus has no RSSI field — GCS heartbeat is the only
        #       authoritative link indicator. tx/rx rates are reported as KVs.
        # ─────────────────────────────────────────────────────────────────
        if self._telemetry_status_msg is None:
            return DiagnosticStatus(level=WARN, name="PX4: Telemetry Link",
                                    message="No messages received", hardware_id="px4_fmu")
        t = self._telemetry_status_msg

        if t.heartbeat_type_gcs:
            level = OK
            msg = f"OK | GCS connected tx={t.tx_rate_avg:.0f}B/s rx={t.rx_rate_avg:.0f}B/s"
        else:
            level = ERROR
            msg = "GCS heartbeat lost"

        kvs = [
            KeyValue(key="heartbeat_type_gcs",   value=str(t.heartbeat_type_gcs)),
            KeyValue(key="tx_rate_avg_bps",       value=f"{t.tx_rate_avg:.1f}"),
            KeyValue(key="rx_rate_avg_bps",       value=f"{t.rx_rate_avg:.1f}"),
            KeyValue(key="rx_message_lost_rate",  value=f"{t.rx_message_lost_rate:.3f}"),
            KeyValue(key="tx_buffer_overruns",    value=str(t.tx_buffer_overruns)),
        ]
        return DiagnosticStatus(level=level, name="PX4: Telemetry Link",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_barometer(self) -> DiagnosticStatus:
        # HEALTH CRITERIA — PX4: Barometer (/fmu/out/vehicle_air_data)
        # ─────────────────────────────────────────────────────────────────
        # OK    : pressure in [baro_pressure_min_pa, baro_pressure_max_pa]
        #         AND not NaN AND rho > 0
        # WARN  : pressure within 5% of the plausibility bounds
        # ERROR : pressure out of bounds OR NaN OR rho <= 0
        # NO MSG: WARN
        # ─────────────────────────────────────────────────────────────────
        if self._air_data_msg is None:
            return DiagnosticStatus(level=WARN, name="PX4: Barometer",
                                    message="No messages received", hardware_id="px4_fmu")
        d = self._air_data_msg
        p_min = self.get_parameter("baro_pressure_min_pa").get_parameter_value().double_value
        p_max = self.get_parameter("baro_pressure_max_pa").get_parameter_value().double_value

        p = d.baro_pressure_pa
        alt = d.baro_alt_meter
        rho = d.rho

        if math.isnan(p) or math.isnan(alt) or math.isnan(rho):
            return DiagnosticStatus(level=ERROR, name="PX4: Barometer",
                                    message="NaN in baro data", hardware_id="px4_fmu")

        margin = 0.05
        if p < p_min or p > p_max or rho <= 0:
            level = ERROR
            msg = f"Out of range: pressure={p:.0f} Pa rho={rho:.3f}"
        elif p < p_min * (1 + margin) or p > p_max * (1 - margin):
            level = WARN
            msg = f"Near bounds: pressure={p:.0f} Pa"
        else:
            level = OK
            msg = f"OK | alt={alt:.1f}m pressure={p:.0f}Pa rho={rho:.3f}"

        kvs = [
            KeyValue(key="baro_alt_meter",    value=f"{alt:.2f}"),
            KeyValue(key="baro_pressure_pa",  value=f"{p:.1f}"),
            KeyValue(key="rho_kg_m3",         value=f"{rho:.4f}"),
        ]
        return DiagnosticStatus(level=level, name="PX4: Barometer",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_home_position(self) -> DiagnosticStatus:
        # HEALTH CRITERIA — PX4: Home Position (/fmu/out/home_position)
        # ─────────────────────────────────────────────────────────────────
        # OK    : valid_hpos=True AND valid_lpos=True AND valid_alt=True
        # WARN  : valid_lpos=True but valid_hpos=False (local home only)
        # ERROR : valid_lpos=False AND valid_alt=False (no home — RTL will fail)
        # NO MSG: WARN
        # ─────────────────────────────────────────────────────────────────
        if self._home_position_msg is None:
            return DiagnosticStatus(level=WARN, name="PX4: Home Position",
                                    message="No messages received", hardware_id="px4_fmu")
        h = self._home_position_msg

        if h.valid_hpos and h.valid_lpos and h.valid_alt:
            level = OK
            msg = "OK | GPS home set"
        elif h.valid_lpos:
            level = WARN
            msg = "Local home only (no GPS home — RTL may use local coords)"
        else:
            level = ERROR
            msg = "No home position (RTL will fail)"

        kvs = [
            KeyValue(key="valid_hpos", value=str(h.valid_hpos)),
            KeyValue(key="valid_lpos", value=str(h.valid_lpos)),
            KeyValue(key="valid_alt",  value=str(h.valid_alt)),
        ]
        return DiagnosticStatus(level=level, name="PX4: Home Position",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_geofence(self) -> DiagnosticStatus:
        # HEALTH CRITERIA — PX4: Geofence (/fmu/out/geofence_result)
        # ─────────────────────────────────────────────────────────────────
        # OK    : none of the three trigger booleans is True
        # ERROR : geofence_max_dist_triggered OR geofence_max_alt_triggered
        #         OR geofence_custom_fence_triggered
        # NO MSG: OK  ← geofence may simply not be configured; absence is normal
        # ─────────────────────────────────────────────────────────────────
        if self._geofence_result_msg is None:
            return DiagnosticStatus(level=OK, name="PX4: Geofence",
                                    message="No geofence configured", hardware_id="px4_fmu")
        gf = self._geofence_result_msg
        action_name = GEOFENCE_ACTION_NAMES.get(gf.geofence_action, str(gf.geofence_action))

        breached_by: list[str] = []
        if gf.geofence_max_dist_triggered:
            breached_by.append("MAX_DIST")
        if gf.geofence_max_alt_triggered:
            breached_by.append("MAX_ALT")
        if gf.geofence_custom_fence_triggered:
            breached_by.append("CUSTOM")

        if breached_by:
            level = ERROR
            msg = f"GEOFENCE BREACHED ({','.join(breached_by)}) | action={action_name}"
        else:
            level = OK
            msg = f"OK | action={action_name}"

        kvs = [
            KeyValue(key="max_dist_triggered",     value=str(gf.geofence_max_dist_triggered)),
            KeyValue(key="max_alt_triggered",      value=str(gf.geofence_max_alt_triggered)),
            KeyValue(key="custom_fence_triggered", value=str(gf.geofence_custom_fence_triggered)),
            KeyValue(key="geofence_action",        value=action_name),
        ]
        return DiagnosticStatus(level=level, name="PX4: Geofence",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    # ------------------------------------------------------------------ #
    # Publish loop                                                         #
    # ------------------------------------------------------------------ #

    def _bp(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

    def _publish_diagnostics(self) -> None:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        statuses = []
        if self._bp("enable_vehicle_status"):   statuses.append(self._diag_vehicle_status())
        if self._bp("enable_failsafe"):         statuses.append(self._diag_failsafe())
        if self._bp("enable_land_detected"):    statuses.append(self._diag_land_detected())
        if self._bp("enable_cpuload"):          statuses.append(self._diag_cpuload())
        if self._bp("enable_timesync"):         statuses.append(self._diag_timesync())
        if self._bp("enable_battery"):          statuses.append(self._diag_battery())
        if self._bp("enable_imu"):              statuses.append(self._diag_imu())
        if self._bp("enable_angular_velocity"): statuses.append(self._diag_ang_vel())
        if self._bp("enable_local_position"):   statuses.append(self._diag_local_position())
        if self._bp("enable_ekf_health"):       statuses.append(self._diag_ekf_health())
        if self._bp("enable_imu_vibration"):    statuses.append(self._diag_imu_vibration())
        if self._bp("enable_ekf_status_flags"): statuses.append(self._diag_ekf_status_flags())
        if self._bp("enable_gps"):              statuses.append(self._diag_gps())
        if self._bp("enable_rc_link"):          statuses.append(self._diag_rc_link())
        if self._bp("enable_attitude"):         statuses.append(self._diag_attitude())
        if self._bp("enable_esc_health"):       statuses.append(self._diag_esc_health())
        if self._bp("enable_telemetry_link"):   statuses.append(self._diag_telemetry_link())
        if self._bp("enable_barometer"):        statuses.append(self._diag_barometer())
        if self._bp("enable_home_position"):    statuses.append(self._diag_home_position())
        if self._bp("enable_geofence"):         statuses.append(self._diag_geofence())
        arr.status = statuses
        self._diag_pub.publish(arr)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DroneDiagnosticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
