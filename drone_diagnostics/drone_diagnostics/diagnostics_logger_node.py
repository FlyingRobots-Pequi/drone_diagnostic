#!/usr/bin/env python3
"""InfluxDB v2 logger for /diagnostics_agg — persists drone diagnostic data for Grafana."""

import os
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

try:
    from influxdb_client import InfluxDBClient
    from influxdb_client.client.write_api import SYNCHRONOUS
    from influxdb_client.domain.write_precision import WritePrecision
    _INFLUX_AVAILABLE = True
except ImportError:
    _INFLUX_AVAILABLE = False

QOS_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

LEVEL_MAP = {
    DiagnosticStatus.OK: 0,
    DiagnosticStatus.WARN: 1,
    DiagnosticStatus.ERROR: 2,
}

# KeyValue keys that should be written as float fields (not tags)
_NUMERIC_KEYS = {
    "actual_hz", "expected_hz", "cpu_percent", "ram_percent", "offset_us",
    "round_trip_time_us", "pos_variance_max", "pos_std_max_m",
    "pos_horiz_accuracy_m", "pos_vert_accuracy_m", "innovation_fail_count",
    "vio_hz", "bridge_hz", "accel_var_x", "accel_var_y", "accel_var_z",
    "accel_var_max", "sample_count", "voltage_v", "remaining_pct", "current_a",
    "discharged_mah", "delta_m", "dx_m", "dy_m", "dz_m", "eph_m", "epv_m",
    "error_count", "warn_count",
}


class DiagnosticsLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("diagnostics_logger")

        self.declare_parameter("influxdb_url", "http://localhost:8086")
        self.declare_parameter("influxdb_org", "aerobatics")
        self.declare_parameter("influxdb_bucket", "drone_diagnostics")
        self.declare_parameter("drone_namespace", "/pequi/hermit")
        self.declare_parameter("batch_interval_sec", 5.0)

        self._url = self.get_parameter("influxdb_url").get_parameter_value().string_value
        self._org = self.get_parameter("influxdb_org").get_parameter_value().string_value
        self._bucket = self.get_parameter("influxdb_bucket").get_parameter_value().string_value
        self._ns = self.get_parameter("drone_namespace").get_parameter_value().string_value
        batch_sec = self.get_parameter("batch_interval_sec").get_parameter_value().double_value

        # Token must come from environment — never from YAML
        self._token = os.environ.get("INFLUXDB_TOKEN", "")

        self._pending: list[dict] = []
        self._client: Optional[InfluxDBClient] = None
        self._write_api = None
        self._backoff_until: float = 0.0

        if not _INFLUX_AVAILABLE:
            self.get_logger().error(
                "influxdb-client not installed — logger node will do nothing. "
                "Run: python3 -m pip install 'influxdb-client>=1.18.0'"
            )
            return

        if not self._token:
            self.get_logger().warn(
                "INFLUXDB_TOKEN env var not set — InfluxDB writes will fail auth"
            )

        self._connect()

        self.create_subscription(DiagnosticArray, "/diagnostics_agg", self._cb_diag, QOS_RELIABLE)
        self.create_timer(batch_sec, self._flush)

        self.get_logger().info(
            f"Diagnostics logger started → {self._url}/{self._bucket} (flush every {batch_sec:.0f}s)"
        )

    def _connect(self) -> None:
        try:
            self._client = InfluxDBClient(
                url=self._url, token=self._token, org=self._org, timeout=5_000
            )
            self._write_api = self._client.write_api(write_options=SYNCHRONOUS)
            self.get_logger().info(f"InfluxDB client connected to {self._url}")
        except Exception as exc:
            self.get_logger().error(f"InfluxDB connect failed: {exc}")
            self._client = None
            self._write_api = None

    def _cb_diag(self, msg: DiagnosticArray) -> None:
        if not _INFLUX_AVAILABLE:
            return

        # Timestamp from header in nanoseconds
        ts_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec

        for status in msg.status:
            name: str = status.name
            # Strip aggregator path prefix (e.g. "/PX4: CPU Load" → "PX4: CPU Load")
            if name.startswith("/"):
                name = name[1:]

            # Derive category from name prefix
            if name.startswith("PX4:"):
                category = "PX4"
            elif name.startswith("VIO:"):
                category = "VIO"
            elif name.startswith("System:"):
                category = "System"
            else:
                category = "Other"

            tags = {
                "drone_namespace": self._ns,
                "category": category,
                "check_name": name,
                "hardware_id": status.hardware_id or "unknown",
                "level": {0: "OK", 1: "WARN", 2: "ERROR"}.get(status.level, "UNKNOWN"),
            }

            fields: dict[str, float | int | str] = {
                "level_numeric": LEVEL_MAP.get(status.level, 0),
            }

            for kv in status.values:
                if kv.key in _NUMERIC_KEYS:
                    try:
                        fields[kv.key] = float(kv.value)
                    except ValueError:
                        pass  # skip non-parseable numeric fields

            self._pending.append({
                "measurement": "drone_diagnostics",
                "tags": tags,
                "fields": fields,
                "time": ts_ns,
            })

    def _flush(self) -> None:
        if not _INFLUX_AVAILABLE or not self._pending:
            return

        if time.monotonic() < self._backoff_until:
            return

        if self._write_api is None:
            self._connect()
            if self._write_api is None:
                self._backoff_until = time.monotonic() + 30.0
                return

        batch = self._pending[:]
        self._pending.clear()

        try:
            self._write_api.write(
                bucket=self._bucket,
                org=self._org,
                record=batch,
                write_precision=WritePrecision.NANOSECONDS,
            )
            self.get_logger().debug(f"Wrote {len(batch)} points to InfluxDB")
        except Exception as exc:
            self.get_logger().warn(f"InfluxDB write failed: {exc} — backing off 30s")
            self._pending = batch + self._pending  # requeue
            self._backoff_until = time.monotonic() + 30.0
            # Force reconnect next flush
            self._write_api = None
            if self._client:
                try:
                    self._client.close()
                except Exception:
                    pass
            self._client = None

    def destroy_node(self) -> None:
        if self._client:
            try:
                self._client.close()
            except Exception:
                pass
        super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DiagnosticsLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
