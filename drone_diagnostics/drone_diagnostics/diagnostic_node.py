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
    EstimatorStatus,
    EstimatorStatusFlags,
    FailsafeFlags,
    SensorCombined,
    TimesyncStatus,
    VehicleAngularVelocity,
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

    # ------------------------------------------------------------------ #
    # Publish loop                                                         #
    # ------------------------------------------------------------------ #

    def _publish_diagnostics(self) -> None:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [
            self._diag_vehicle_status(),
            self._diag_failsafe(),
            self._diag_land_detected(),
            self._diag_cpuload(),
            self._diag_timesync(),
            self._diag_battery(),
            self._diag_imu(),
            self._diag_ang_vel(),
            self._diag_local_position(),
            self._diag_ekf_health(),
            self._diag_imu_vibration(),
            self._diag_ekf_status_flags(),
        ]
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
