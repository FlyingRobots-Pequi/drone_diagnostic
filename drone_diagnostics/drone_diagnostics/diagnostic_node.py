#!/usr/bin/env python3
"""Real-time drone diagnostic node for PX4 + OpenVINS MSCKF system."""

import math
import time
from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    BatteryStatus,
    Cpuload,
    EstimatorStatus,
    EstimatorStatusFlags,
    FailsafeFlags,
    OffboardControlMode,
    SensorCombined,
    TimesyncStatus,
    TrajectorySetpoint,
    VehicleAngularVelocity,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleOdometry,
    VehicleStatus,
)
from rcl_interfaces.msg import Log

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

QOS_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
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

# EstimatorStatus.innovation_check_flags bit meanings
INNOV_BITS = [
    "vel_horiz", "vel_vert", "pos_horiz", "pos_vert",
    "magnetometer", "heading", "airspeed", "beta",
    "hagl", "optflow_x", "optflow_y", "gravity",
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

    def rate_status(self) -> tuple[int, str]:
        if not self.is_alive:
            msg = "No messages" if self._last_msg_time is None else f"Stale ({self.staleness_sec:.1f}s)"
            return ERROR, msg
        ratio = self.actual_hz / self.expected_hz if self.expected_hz > 0 else 1.0
        if ratio < 0.5:
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

        # Externalized thresholds (A2)
        self.declare_parameter("max_pos_variance_m2", 0.25)
        self.declare_parameter("max_vel_variance_m2s2", 0.10)
        self.declare_parameter("cpu_warn_pct", 0.70)
        self.declare_parameter("cpu_error_pct", 0.90)
        self.declare_parameter("ram_warn_pct", 0.80)
        self.declare_parameter("timesync_warn_us", 500)
        self.declare_parameter("timesync_error_us", 2000)

        ns = self.get_parameter("drone_namespace").get_parameter_value().string_value
        timeout = self.get_parameter("topic_timeout_sec").get_parameter_value().double_value

        # ------------------------------------------------------------------ #
        # Topic monitors (name, expected_hz, timeout)                         #
        # ------------------------------------------------------------------ #

        # fmu/in — commands going to PX4
        self._mon_offboard = TopicMonitor(f"{ns}/fmu/in/offboard_control_mode", 10.0, timeout)
        self._mon_vis_odom_in = TopicMonitor(f"{ns}/fmu/in/vehicle_visual_odometry", 30.0, timeout)
        self._mon_traj = TopicMonitor(f"{ns}/fmu/in/trajectory_setpoint", 10.0, timeout)

        # fmu/out — telemetry from PX4
        self._mon_imu = TopicMonitor(f"{ns}/fmu/out/sensor_combined", 100.0, timeout)
        self._mon_ang_vel = TopicMonitor(f"{ns}/fmu/out/vehicle_angular_velocity", 100.0, timeout)
        self._mon_local_pos = TopicMonitor(f"{ns}/fmu/out/vehicle_local_position", 100.0, timeout)
        self._mon_attitude = TopicMonitor(f"{ns}/fmu/out/vehicle_attitude", 100.0, timeout)
        self._mon_vehicle_odom = TopicMonitor(f"{ns}/fmu/out/vehicle_odometry", 100.0, timeout)
        self._mon_estimator = TopicMonitor(f"{ns}/fmu/out/estimator_status", 100.0, timeout)

        # VIO — OpenVINS
        self._mon_vio_pose = TopicMonitor("/ov_msckf/poseimu", 30.0, timeout)
        self._mon_vio_odom = TopicMonitor("/ov_msckf/odomimu", 30.0, timeout)

        # ------------------------------------------------------------------ #
        # Latest message state                                                 #
        # ------------------------------------------------------------------ #

        self._offboard_msg: Optional[OffboardControlMode] = None
        self._vis_odom_in_msg: Optional[VehicleOdometry] = None
        self._vehicle_status_msg: Optional[VehicleStatus] = None
        self._estimator_msg: Optional[EstimatorStatus] = None
        self._estimator_flags_msg: Optional[EstimatorStatusFlags] = None
        self._failsafe_msg: Optional[FailsafeFlags] = None
        self._cpuload_msg: Optional[Cpuload] = None
        self._timesync_msg: Optional[TimesyncStatus] = None
        self._land_detected_msg: Optional[VehicleLandDetected] = None
        self._local_pos_msg: Optional[VehicleLocalPosition] = None
        self._vio_pose_msg: Optional[PoseWithCovarianceStamped] = None
        self._battery_msg: Optional[BatteryStatus] = None

        # Accel samples for vibration analysis (A1)
        self._accel_samples: deque[tuple[float, float, float]] = deque(maxlen=30)

        self._rosout_errors: deque[str] = deque(maxlen=5)
        self._rosout_warn_count: int = 0
        self._rosout_error_count: int = 0

        # ------------------------------------------------------------------ #
        # Subscriptions                                                        #
        # ------------------------------------------------------------------ #

        # fmu/in
        self.create_subscription(OffboardControlMode, f"{ns}/fmu/in/offboard_control_mode", self._cb_offboard, QOS_PX4)
        self.create_subscription(VehicleOdometry, f"{ns}/fmu/in/vehicle_visual_odometry", self._cb_vis_odom_in, QOS_PX4)
        self.create_subscription(TrajectorySetpoint, f"{ns}/fmu/in/trajectory_setpoint", lambda _: self._mon_traj.on_message(), QOS_PX4)

        # fmu/out — high-rate
        self.create_subscription(SensorCombined, f"{ns}/fmu/out/sensor_combined", self._cb_sensor_combined, QOS_PX4)
        self.create_subscription(VehicleAngularVelocity, f"{ns}/fmu/out/vehicle_angular_velocity", lambda _: self._mon_ang_vel.on_message(), QOS_PX4)
        self.create_subscription(VehicleLocalPosition, f"{ns}/fmu/out/vehicle_local_position", self._cb_local_pos, QOS_PX4)
        self.create_subscription(VehicleOdometry, f"{ns}/fmu/out/vehicle_odometry", lambda _: self._mon_vehicle_odom.on_message(), QOS_PX4)
        self.create_subscription(EstimatorStatus, f"{ns}/fmu/out/estimator_status", self._cb_estimator, QOS_PX4)

        # fmu/out — low-rate status
        self.create_subscription(VehicleStatus, f"{ns}/fmu/out/vehicle_status", self._cb_vehicle_status, QOS_PX4)
        self.create_subscription(EstimatorStatusFlags, f"{ns}/fmu/out/estimator_status_flags", self._cb_estimator_flags, QOS_PX4)
        self.create_subscription(FailsafeFlags, f"{ns}/fmu/out/failsafe_flags", self._cb_failsafe, QOS_PX4)
        self.create_subscription(Cpuload, f"{ns}/fmu/out/cpuload", self._cb_cpuload, QOS_PX4)
        self.create_subscription(TimesyncStatus, f"{ns}/fmu/out/timesync_status", self._cb_timesync, QOS_PX4)
        self.create_subscription(VehicleLandDetected, f"{ns}/fmu/out/vehicle_land_detected", self._cb_land_detected, QOS_PX4)
        self.create_subscription(BatteryStatus, f"{ns}/fmu/out/battery_status", self._cb_battery, QOS_PX4)

        # VIO
        self.create_subscription(PoseWithCovarianceStamped, "/ov_msckf/poseimu", self._cb_vio_pose, QOS_RELIABLE)
        self.create_subscription(Odometry, "/ov_msckf/odomimu", lambda _: self._mon_vio_odom.on_message(), QOS_RELIABLE)

        # Rosout
        self.create_subscription(Log, "/rosout", self._cb_rosout, QOS_RELIABLE)

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

    def _cb_offboard(self, msg: OffboardControlMode) -> None:
        self._mon_offboard.on_message()
        self._offboard_msg = msg

    def _cb_vis_odom_in(self, msg: VehicleOdometry) -> None:
        self._mon_vis_odom_in.on_message()
        self._vis_odom_in_msg = msg

    def _cb_local_pos(self, msg: VehicleLocalPosition) -> None:
        self._mon_local_pos.on_message()
        self._local_pos_msg = msg

    def _cb_estimator(self, msg: EstimatorStatus) -> None:
        self._mon_estimator.on_message()
        self._estimator_msg = msg

    def _cb_vehicle_status(self, msg: VehicleStatus) -> None:
        self._vehicle_status_msg = msg

    def _cb_estimator_flags(self, msg: EstimatorStatusFlags) -> None:
        self._estimator_flags_msg = msg

    def _cb_failsafe(self, msg: FailsafeFlags) -> None:
        self._failsafe_msg = msg

    def _cb_cpuload(self, msg: Cpuload) -> None:
        self._cpuload_msg = msg

    def _cb_timesync(self, msg: TimesyncStatus) -> None:
        self._timesync_msg = msg

    def _cb_land_detected(self, msg: VehicleLandDetected) -> None:
        self._land_detected_msg = msg

    def _cb_vio_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._mon_vio_pose.on_message()
        self._vio_pose_msg = msg

    def _cb_sensor_combined(self, msg: SensorCombined) -> None:
        self._mon_imu.on_message()
        self._accel_samples.append((msg.accelerometer_m_s2[0],
                                    msg.accelerometer_m_s2[1],
                                    msg.accelerometer_m_s2[2]))

    def _cb_battery(self, msg: BatteryStatus) -> None:
        self._battery_msg = msg

    _LOG_WARN = 30
    _LOG_ERROR = 40

    def _cb_rosout(self, msg: Log) -> None:
        if msg.level >= self._LOG_ERROR:
            self._rosout_error_count += 1
            self._rosout_errors.append(f"[{msg.name}] {msg.msg[:80]}")
        elif msg.level >= self._LOG_WARN:
            self._rosout_warn_count += 1

    # ------------------------------------------------------------------ #
    # Diagnostic builders — fmu/in & commands                             #
    # ------------------------------------------------------------------ #

    def _diag_px4_offboard(self) -> DiagnosticStatus:
        level, msg = self._mon_offboard.rate_status()
        kvs = self._mon_offboard.base_kvs()
        if self._offboard_msg is not None:
            m = self._offboard_msg
            modes = [n for n, f in [("position", m.position), ("velocity", m.velocity),
                                     ("acceleration", m.acceleration), ("attitude", m.attitude),
                                     ("body_rate", m.body_rate)] if f]
            kvs.append(KeyValue(key="active_modes", value=",".join(modes) or "none"))
        return DiagnosticStatus(level=level, name="PX4: Offboard Control Mode",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_px4_vio_bridge(self) -> DiagnosticStatus:
        max_pos_var = self.get_parameter("max_pos_variance_m2").get_parameter_value().double_value
        level, msg = self._mon_vis_odom_in.rate_status()
        kvs = self._mon_vis_odom_in.base_kvs()
        if self._vis_odom_in_msg is not None and self._mon_vis_odom_in.is_alive:
            vo = self._vis_odom_in_msg
            pos_var = list(vo.position_variance)
            vel_var = list(vo.velocity_variance)
            max_pv = max(pos_var)
            kvs += [
                KeyValue(key="pos_variance_xyz", value=f"{pos_var[0]:.4f},{pos_var[1]:.4f},{pos_var[2]:.4f}"),
                KeyValue(key="vel_variance_xyz", value=f"{vel_var[0]:.4f},{vel_var[1]:.4f},{vel_var[2]:.4f}"),
                KeyValue(key="pos_std_max_m", value=f"{math.sqrt(max(0.0, max_pv)):.3f}"),
                KeyValue(key="pos_variance_max", value=f"{max_pv:.4f}"),
            ]
            if max_pv > max_pos_var and level == OK:
                level, msg = WARN, f"High position variance ({math.sqrt(max_pv):.2f}m std)"
        return DiagnosticStatus(level=level, name="PX4: Visual Odometry Bridge",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_px4_trajectory(self) -> DiagnosticStatus:
        level, msg = self._mon_traj.rate_status()
        return DiagnosticStatus(level=level, name="PX4: Trajectory Setpoint",
                                message=msg, hardware_id="px4_fmu", values=self._mon_traj.base_kvs())

    # ------------------------------------------------------------------ #
    # Diagnostic builders — fmu/out telemetry                             #
    # ------------------------------------------------------------------ #

    def _diag_vehicle_status(self) -> DiagnosticStatus:
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
        elif s.arming_state not in (1, 2):  # not DISARMED or ARMED
            level = WARN

        kvs = [
            KeyValue(key="arming_state", value=arm),
            KeyValue(key="nav_state", value=nav),
            KeyValue(key="failsafe", value=str(s.failsafe)),
        ]
        return DiagnosticStatus(level=level, name="PX4: Vehicle Status",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_ekf_health(self) -> DiagnosticStatus:
        level, msg = self._mon_estimator.rate_status()
        kvs = self._mon_estimator.base_kvs()
        if self._estimator_msg is not None and self._mon_estimator.is_alive:
            e = self._estimator_msg
            failed_bits = [INNOV_BITS[i] for i in range(len(INNOV_BITS))
                           if (e.innovation_check_flags >> i) & 1]
            innov_fail_count = len(failed_bits)
            kvs += [
                KeyValue(key="pos_horiz_accuracy_m", value=f"{e.pos_horiz_accuracy:.3f}"),
                KeyValue(key="pos_vert_accuracy_m", value=f"{e.pos_vert_accuracy:.3f}"),
                KeyValue(key="innovation_failures", value=",".join(failed_bits) or "none"),
                KeyValue(key="innovation_fail_count", value=str(innov_fail_count)),
            ]
            if failed_bits and level == OK:
                level = WARN
                msg = f"Innovation failures: {','.join(failed_bits)}"
            if e.pos_horiz_accuracy > 5.0 and level == OK:
                level = WARN
                msg = f"Low horizontal accuracy ({e.pos_horiz_accuracy:.1f}m)"
        return DiagnosticStatus(level=level, name="PX4: EKF Health",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_failsafe(self) -> DiagnosticStatus:
        if self._failsafe_msg is None:
            return DiagnosticStatus(level=WARN, name="PX4: Failsafe Flags",
                                    message="No messages received", hardware_id="px4_fmu")
        f = self._failsafe_msg
        active = [
            field for field in [
                "angular_velocity_invalid", "attitude_invalid",
                "local_altitude_invalid", "local_position_invalid",
                "local_position_invalid_relaxed", "local_velocity_invalid",
                "global_position_invalid", "offboard_control_signal_lost",
                "battery_warning", "battery_unhealthy",
                "geofence_breached", "local_position_accuracy_low",
                "mission_failure",
            ]
            if getattr(f, field, False)
        ]
        level = ERROR if active else OK
        msg = f"Active: {','.join(active)}" if active else "No active failsafes"
        kvs = [KeyValue(key=field, value="TRUE") for field in active]
        return DiagnosticStatus(level=level, name="PX4: Failsafe Flags",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_cpuload(self) -> DiagnosticStatus:
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

    def _diag_land_detected(self) -> DiagnosticStatus:
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

    def _diag_imu(self) -> DiagnosticStatus:
        level, msg = self._mon_imu.rate_status()
        return DiagnosticStatus(level=level, name="PX4: IMU (sensor_combined)",
                                message=msg, hardware_id="px4_fmu", values=self._mon_imu.base_kvs())

    def _diag_local_position(self) -> DiagnosticStatus:
        level, msg = self._mon_local_pos.rate_status()
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

    def _diag_battery(self) -> DiagnosticStatus:
        """4S LiPo: WARN <14.4V / ERROR <13.6V; WARN <25% / ERROR <10% remaining."""
        if self._battery_msg is None:
            return DiagnosticStatus(level=WARN, name="PX4: Battery Health",
                                    message="No messages received", hardware_id="px4_fmu")
        b = self._battery_msg
        voltage = b.voltage_v
        remaining = b.remaining  # 0.0–1.0
        current = b.current_a
        remaining_pct = remaining * 100.0

        if voltage < 13.6 or remaining < 0.10:
            level = ERROR
            msg = f"CRITICAL: {voltage:.2f}V {remaining_pct:.0f}%"
        elif voltage < 14.4 or remaining < 0.25:
            level = WARN
            msg = f"Low battery: {voltage:.2f}V {remaining_pct:.0f}%"
        else:
            level = OK
            msg = f"{voltage:.2f}V {remaining_pct:.0f}% ({current:.1f}A)"

        kvs = [
            KeyValue(key="voltage_v", value=f"{voltage:.3f}"),
            KeyValue(key="remaining_pct", value=f"{remaining_pct:.1f}"),
            KeyValue(key="current_a", value=f"{current:.2f}"),
            KeyValue(key="discharged_mah", value=f"{b.discharged_mah:.0f}"),
        ]
        return DiagnosticStatus(level=level, name="PX4: Battery Health",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_imu_vibration(self) -> DiagnosticStatus:
        """Compute accelerometer variance over last 30 samples as vibration indicator."""
        if len(self._accel_samples) < 10:
            return DiagnosticStatus(
                level=WARN, name="PX4: IMU Vibration",
                message="Insufficient samples", hardware_id="px4_fmu",
                values=[KeyValue(key="sample_count", value=str(len(self._accel_samples)))]
            )
        samples = list(self._accel_samples)
        n = len(samples)
        ax = [s[0] for s in samples]
        ay = [s[1] for s in samples]
        az = [s[2] for s in samples]

        def variance(vals: list[float]) -> float:
            mean = sum(vals) / len(vals)
            return sum((v - mean) ** 2 for v in vals) / len(vals)

        vx, vy, vz = variance(ax), variance(ay), variance(az)
        max_var = max(vx, vy, vz)

        # Thresholds: >4 m²/s⁴ indicates significant vibration
        if max_var > 9.0:
            level = ERROR
            msg = f"High vibration: {max_var:.2f} m²/s⁴"
        elif max_var > 4.0:
            level = WARN
            msg = f"Elevated vibration: {max_var:.2f} m²/s⁴"
        else:
            level = OK
            msg = f"OK ({max_var:.2f} m²/s⁴)"

        kvs = [
            KeyValue(key="accel_var_x", value=f"{vx:.4f}"),
            KeyValue(key="accel_var_y", value=f"{vy:.4f}"),
            KeyValue(key="accel_var_z", value=f"{vz:.4f}"),
            KeyValue(key="accel_var_max", value=f"{max_var:.4f}"),
            KeyValue(key="sample_count", value=str(n)),
        ]
        return DiagnosticStatus(level=level, name="PX4: IMU Vibration",
                                message=msg, hardware_id="px4_fmu", values=kvs)

    def _diag_vio_ekf_agreement(self) -> DiagnosticStatus:
        """Compare VIO XYZ position vs EKF local position — warn if divergence > 0.5m."""
        if self._vio_pose_msg is None or not self._mon_vio_pose.is_alive:
            return DiagnosticStatus(
                level=WARN, name="VIO: VIO↔EKF Agreement",
                message="VIO pose unavailable", hardware_id="ov_msckf"
            )
        if self._local_pos_msg is None or not self._mon_local_pos.is_alive:
            return DiagnosticStatus(
                level=WARN, name="VIO: VIO↔EKF Agreement",
                message="EKF local position unavailable", hardware_id="ov_msckf"
            )

        vp = self._vio_pose_msg.pose.pose.position
        lp = self._local_pos_msg
        dx = vp.x - lp.x
        dy = vp.y - lp.y
        dz = vp.z - lp.z
        delta = math.sqrt(dx * dx + dy * dy + dz * dz)

        if delta > 1.0:
            level = ERROR
            msg = f"Large divergence: {delta:.2f}m"
        elif delta > 0.5:
            level = WARN
            msg = f"VIO↔EKF diverging: {delta:.2f}m"
        else:
            level = OK
            msg = f"Aligned ({delta:.2f}m)"

        kvs = [
            KeyValue(key="delta_m", value=f"{delta:.3f}"),
            KeyValue(key="dx_m", value=f"{dx:.3f}"),
            KeyValue(key="dy_m", value=f"{dy:.3f}"),
            KeyValue(key="dz_m", value=f"{dz:.3f}"),
        ]
        return DiagnosticStatus(level=level, name="VIO: VIO↔EKF Agreement",
                                message=msg, hardware_id="ov_msckf", values=kvs)

    # ------------------------------------------------------------------ #
    # Diagnostic builders — VIO                                           #
    # ------------------------------------------------------------------ #

    def _diag_vio_pipeline(self) -> DiagnosticStatus:
        vio_alive = self._mon_vio_pose.is_alive or self._mon_vio_odom.is_alive
        bridge_alive = self._mon_vis_odom_in.is_alive
        if vio_alive and not bridge_alive:
            level, msg = ERROR, "VIO running but NOT reaching PX4"
        elif not vio_alive and not bridge_alive:
            level, msg = ERROR, "VIO pipeline completely offline"
        elif not vio_alive and bridge_alive:
            level, msg = WARN, "VIO pose lost — bridge has stale data"
        else:
            level, msg = OK, "VIO -> PX4 bridge OK"
        return DiagnosticStatus(
            level=level, name="VIO: Pipeline Health", message=msg, hardware_id="ov_msckf",
            values=[
                KeyValue(key="vio_alive", value=str(vio_alive)),
                KeyValue(key="px4_bridge_alive", value=str(bridge_alive)),
                KeyValue(key="vio_hz", value=f"{self._mon_vio_pose.actual_hz:.2f}"),
                KeyValue(key="bridge_hz", value=f"{self._mon_vis_odom_in.actual_hz:.2f}"),
            ],
        )

    def _diag_vio_pose(self) -> DiagnosticStatus:
        max_pos_var = self.get_parameter("max_pos_variance_m2").get_parameter_value().double_value
        level, msg = self._mon_vio_pose.rate_status()
        kvs = self._mon_vio_pose.base_kvs()
        if self._vio_pose_msg is not None and self._mon_vio_pose.is_alive:
            cov = self._vio_pose_msg.pose.covariance
            pv_x, pv_y, pv_z = cov[0], cov[7], cov[14]
            max_pv = max(pv_x, pv_y, pv_z)
            kvs += [
                KeyValue(key="pos_cov_diag", value=f"{pv_x:.4f},{pv_y:.4f},{pv_z:.4f}"),
                KeyValue(key="pos_std_max_m", value=f"{math.sqrt(max(0.0, max_pv)):.3f}"),
            ]
            if max_pv > max_pos_var and level == OK:
                level = WARN
                msg = f"High position covariance ({math.sqrt(max_pv):.2f}m std)"
        return DiagnosticStatus(level=level, name="VIO: OpenVINS Pose (poseimu)",
                                message=msg, hardware_id="ov_msckf", values=kvs)

    def _diag_vio_odometry(self) -> DiagnosticStatus:
        level, msg = self._mon_vio_odom.rate_status()
        return DiagnosticStatus(level=level, name="VIO: OpenVINS Odometry (odomimu)",
                                message=msg, hardware_id="ov_msckf", values=self._mon_vio_odom.base_kvs())

    # ------------------------------------------------------------------ #
    # Diagnostic builders — system                                        #
    # ------------------------------------------------------------------ #

    def _diag_rosout(self) -> DiagnosticStatus:
        if self._rosout_error_count > 0:
            level, msg = ERROR, f"{self._rosout_error_count} errors logged"
        elif self._rosout_warn_count > 0:
            level, msg = WARN, f"{self._rosout_warn_count} warnings logged"
        else:
            level, msg = OK, "No errors or warnings"
        kvs = [
            KeyValue(key="error_count", value=str(self._rosout_error_count)),
            KeyValue(key="warn_count", value=str(self._rosout_warn_count)),
        ]
        for i, err in enumerate(self._rosout_errors):
            kvs.append(KeyValue(key=f"last_error_{i}", value=err))
        return DiagnosticStatus(level=level, name="System: ROS Log",
                                message=msg, hardware_id="rosout", values=kvs)

    # ------------------------------------------------------------------ #
    # Publish loop                                                         #
    # ------------------------------------------------------------------ #

    def _publish_diagnostics(self) -> None:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [
            # VIO pipeline (aggregate — shown first as it's most critical)
            self._diag_vio_pipeline(),
            # PX4 fmu/in — commands
            self._diag_px4_offboard(),
            self._diag_px4_vio_bridge(),
            self._diag_px4_trajectory(),
            # PX4 fmu/out — telemetry
            self._diag_vehicle_status(),
            self._diag_ekf_health(),
            self._diag_failsafe(),
            self._diag_local_position(),
            self._diag_imu(),
            self._diag_imu_vibration(),
            self._diag_cpuload(),
            self._diag_timesync(),
            self._diag_land_detected(),
            self._diag_battery(),
            # VIO
            self._diag_vio_pose(),
            self._diag_vio_odometry(),
            self._diag_vio_ekf_agreement(),
            # System
            self._diag_rosout(),
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
