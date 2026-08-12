#!/usr/bin/env python3
"""InfluxDB v2 logger for /diagnostics_agg — persists drone diagnostic data.

Points are queued in a local SQLite file (not just RAM) before being written
to InfluxDB. This makes the logger resilient to prolonged network/InfluxDB
outages — a real scenario in the field (drone out of telemetry range, GCS
disconnected, InfluxDB down) — without risking unbounded memory growth on
the companion computer, and without losing the last points before a power
loss (exactly the moment a post-incident log matters most).
"""

import json
import os
import sqlite3
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

try:
    from influxdb_client.client.influxdb_client import InfluxDBClient
    from influxdb_client.client.write_api import SYNCHRONOUS
    _INFLUX_AVAILABLE = True
except ImportError:
    InfluxDBClient = None  # type: ignore[assignment,misc]
    SYNCHRONOUS = None  # type: ignore[assignment]
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

# String labels indexed by the *numeric* level (0/1/2) — used to build the
# "level" tag from the already-correct LEVEL_MAP lookup below. Do not compare
# status.level (a `bytes` value in rclpy) directly against int keys — that
# comparison silently never matches and always falls through to a default.
LEVEL_LABELS = {0: "OK", 1: "WARN", 2: "ERROR"}

# Points are queued locally before being written; this bounds how many can
# accumulate during an outage so a companion computer never OOMs because
# InfluxDB is unreachable. ~50k rows covers hours of outage at 2Hz x ~20
# checks and costs a few MB of SQLite storage.
DEFAULT_MAX_PENDING_POINTS = 50_000
DEFAULT_PENDING_DB_PATH = "/var/lib/drone_diagnostics/pending.db"

# KeyValue keys that should be written as float fields (not tags)
_NUMERIC_KEYS = {
    "actual_hz", "expected_hz", "cpu_percent", "ram_percent", "offset_us",
    "round_trip_time_us", "pos_variance_max", "pos_std_max_m",
    "pos_horiz_accuracy_m", "pos_vert_accuracy_m", "innovation_fail_count",
    "vio_hz", "bridge_hz", "accel_var_x", "accel_var_y", "accel_var_z",
    "accel_var_max", "sample_count", "voltage_v", "remaining_pct", "current_a",
    "discharged_mah", "delta_m", "dx_m", "dy_m", "dz_m", "eph_m", "epv_m",
    "error_count", "warn_count",
    # Post-incident forensics — without these, InfluxDB only records the
    # OK/WARN/ERROR level of "PX4: IMU Vibration"/"PX4: Vehicle Status", not
    # the underlying values needed to reconstruct what actually happened.
    "accel_vibration_metric_ms2", "gyro_vibration_metric_rads",
}

# KeyValue keys written as a "0x.." hex string by diagnostic_node.py — these
# need int(value, 16) instead of the float(value) used for _NUMERIC_KEYS.
_HEX_KEYS = {"failure_detector"}

# KeyValue keys written as "True"/"False" (str(bool)) that are worth having
# as a numeric 0/1 field for querying/alerting (e.g. freefall over time).
_BOOL_KEYS = {"freefall"}


class DiagnosticsLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("diagnostics_logger")

        self.declare_parameter("influxdb_url", "http://localhost:8086")
        self.declare_parameter("influxdb_org", "aerobatics")
        self.declare_parameter("influxdb_bucket", "drone_diagnostics")
        self.declare_parameter("drone_namespace", "/pequi/hermit")
        self.declare_parameter("batch_interval_sec", 5.0)
        self.declare_parameter("batch_write_size", 500)
        self.declare_parameter("pending_db_path", DEFAULT_PENDING_DB_PATH)
        self.declare_parameter("max_pending_points", DEFAULT_MAX_PENDING_POINTS)

        self._url = self.get_parameter("influxdb_url").get_parameter_value().string_value
        self._org = self.get_parameter("influxdb_org").get_parameter_value().string_value
        self._bucket = self.get_parameter("influxdb_bucket").get_parameter_value().string_value
        self._ns = self.get_parameter("drone_namespace").get_parameter_value().string_value
        batch_sec = self.get_parameter("batch_interval_sec").get_parameter_value().double_value
        self._batch_write_size = self.get_parameter("batch_write_size").get_parameter_value().integer_value
        self._pending_db_path = self.get_parameter("pending_db_path").get_parameter_value().string_value
        self._max_pending_points = (
            self.get_parameter("max_pending_points").get_parameter_value().integer_value
        )

        # Token must come from environment — never from YAML
        self._token = os.environ.get("INFLUXDB_TOKEN", "")

        self._client: Optional[InfluxDBClient] = None
        self._write_api = None
        self._backoff_until: float = 0.0

        self._db = self._open_pending_db(self._pending_db_path)

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

        pending_count = self._pending_count()
        self.get_logger().info(
            f"Diagnostics logger started → {self._url}/{self._bucket} (flush every {batch_sec:.0f}s, "
            f"pending queue: {self._pending_db_path}, {pending_count} point(s) already queued)"
        )

    def _open_pending_db(self, path: str) -> sqlite3.Connection:
        """Open (creating if needed) the local durable queue of unsent points.

        WAL mode + synchronous=NORMAL gives a good durability/throughput
        tradeoff for a ~40 msg/s workload — points survive a process crash,
        without paying a full fsync on every single insert.
        """
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        conn = sqlite3.connect(path, isolation_level=None, check_same_thread=False)
        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute("PRAGMA synchronous=NORMAL")
        conn.execute(
            "CREATE TABLE IF NOT EXISTS pending_points ("
            "id INTEGER PRIMARY KEY AUTOINCREMENT, "
            "payload TEXT NOT NULL, "
            "created_at REAL NOT NULL"
            ")"
        )
        return conn

    def _pending_count(self) -> int:
        row = self._db.execute("SELECT COUNT(*) FROM pending_points").fetchone()
        return row[0] if row else 0

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

            # level_numeric first: status.level is `bytes` in rclpy, so it
            # must go through LEVEL_MAP (keyed by the DiagnosticStatus.*
            # constants, also `bytes`) — never compare it directly against
            # int keys, that silently never matches (see LEVEL_LABELS above).
            level_numeric = LEVEL_MAP.get(status.level, 0)

            tags = {
                "drone_namespace": self._ns,
                "category": category,
                "check_name": name,
                "hardware_id": status.hardware_id or "unknown",
                "level": LEVEL_LABELS.get(level_numeric, "UNKNOWN"),
            }

            fields: dict[str, float | int | str] = {
                "level_numeric": level_numeric,
            }

            for kv in status.values:
                if kv.key in _NUMERIC_KEYS:
                    try:
                        fields[kv.key] = float(kv.value)
                    except ValueError:
                        pass  # skip non-parseable numeric fields
                elif kv.key in _HEX_KEYS:
                    try:
                        fields[f"{kv.key}_bitmask"] = int(kv.value, 16)
                    except ValueError:
                        pass
                elif kv.key in _BOOL_KEYS:
                    fields[kv.key] = 1 if kv.value == "True" else 0

            self._enqueue({
                "measurement": "drone_diagnostics",
                "tags": tags,
                "fields": fields,
                "time": ts_ns,
            })

    def _enqueue(self, point: dict) -> None:
        """Durably queue one point, then trim the queue if it grew past the cap."""
        now = time.time()
        self._db.execute(
            "INSERT INTO pending_points (payload, created_at) VALUES (?, ?)",
            (json.dumps(point), now),
        )
        count = self._pending_count()
        overflow = count - self._max_pending_points
        if overflow > 0:
            # Drop the oldest rows first — an outage long enough to hit the
            # cap means the newest data is the most actionable to keep.
            self._db.execute(
                "DELETE FROM pending_points WHERE id IN "
                "(SELECT id FROM pending_points ORDER BY id ASC LIMIT ?)",
                (overflow,),
            )
            self.get_logger().warn(
                f"Pending queue exceeded {self._max_pending_points} points — "
                f"dropped {overflow} oldest point(s). InfluxDB has been unreachable "
                f"for a long time."
            )

    def _flush(self) -> None:
        if not _INFLUX_AVAILABLE:
            return

        if time.monotonic() < self._backoff_until:
            return

        rows = self._db.execute(
            "SELECT id, payload FROM pending_points ORDER BY id ASC LIMIT ?",
            (self._batch_write_size,),
        ).fetchall()
        if not rows:
            return

        if self._write_api is None:
            self._connect()
            if self._write_api is None:
                self._backoff_until = time.monotonic() + 30.0
                return

        ids = [r[0] for r in rows]
        batch = [json.loads(r[1]) for r in rows]

        try:
            self._write_api.write(
                bucket=self._bucket,
                org=self._org,
                record=batch,
                write_precision="ns",
            )
            # Only remove points once InfluxDB has confirmed the write —
            # the queue is the source of truth until then, so a crash or
            # write failure between here and the next flush loses nothing.
            self._db.execute(
                f"DELETE FROM pending_points WHERE id IN ({','.join('?' * len(ids))})",
                ids,
            )
            self.get_logger().debug(f"Wrote {len(batch)} points to InfluxDB")
        except Exception as exc:
            self.get_logger().warn(f"InfluxDB write failed: {exc} — backing off 30s")
            self._backoff_until = time.monotonic() + 30.0
            # Force reconnect next flush. Rows stay queued in SQLite — no
            # in-memory requeue needed.
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
        if getattr(self, "_db", None):
            try:
                self._db.close()
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
