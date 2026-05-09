#!/usr/bin/env python3
"""
ULG Flight Log Ingestor
Watches px4_logs/ for new .ulg files and writes full telemetry to InfluxDB
bucket 'px4_flight_logs' (infinite retention, separate from live diagnostics).

Usage:
    cd <project-root>
    pip install -r ulg_ingestor/requirements.txt
    python ulg_ingestor/ulg_ingestor.py
"""

import json
import logging
import os
import re
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from dotenv import load_dotenv
from influxdb_client.client.influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS
from influxdb_client.domain.bucket_retention_rules import BucketRetentionRules
from pyulog import ULog  # type: ignore[import-untyped]
from watchdog.events import FileSystemEventHandler
from watchdog.observers import Observer

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

_PROJECT_ROOT = Path(__file__).parent.parent
load_dotenv(_PROJECT_ROOT / ".env")

INFLUXDB_URL    = os.getenv("INFLUXDB_URL", "http://localhost:8086")
INFLUXDB_TOKEN  = os.getenv("INFLUXDB_TOKEN", "")
INFLUXDB_ORG    = os.getenv("INFLUXDB_ORG", "aerobatics")
INFLUXDB_BUCKET = "px4_flight_logs"
WATCH_DIR       = Path(os.getenv("ULG_WATCH_DIR", str(_PROJECT_ROOT / "px4_logs")))
BATCH_SIZE      = 5_000
STATE_FILE      = Path(os.getenv("ULG_STATE_FILE", str(Path(__file__).parent / ".ingested_logs.json")))

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s  %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
log = logging.getLogger("ulg_ingestor")

# ---------------------------------------------------------------------------
# State persistence
# ---------------------------------------------------------------------------

def load_state() -> dict:
    if STATE_FILE.exists():
        try:
            return json.loads(STATE_FILE.read_text())
        except Exception:
            return {}
    return {}


def save_state(state: dict) -> None:
    STATE_FILE.write_text(json.dumps(state, indent=2))

# ---------------------------------------------------------------------------
# InfluxDB helpers
# ---------------------------------------------------------------------------

def ensure_bucket(client: InfluxDBClient) -> None:
    api = client.buckets_api()
    existing = api.find_buckets(name=INFLUXDB_BUCKET).buckets
    if existing:
        log.info("Bucket '%s' already exists", INFLUXDB_BUCKET)
        return
    org_id = client.organizations_api().find_organizations(org=INFLUXDB_ORG)[0].id
    api.create_bucket(
        bucket_name=INFLUXDB_BUCKET,
        retention_rules=BucketRetentionRules(type="expire", every_seconds=0),
        org_id=org_id,
    )
    log.info("Created bucket '%s' (infinite retention)", INFLUXDB_BUCKET)

# ---------------------------------------------------------------------------
# ULG parsing helpers
# ---------------------------------------------------------------------------

def sanitize_field(name: str) -> str:
    return name.replace("[", "_").replace("]", "").rstrip("_")


def extract_drone_name(ulg_path: Path) -> str:
    try:
        return ulg_path.relative_to(WATCH_DIR).parts[0]
    except (ValueError, IndexError):
        return "unknown"


_FILENAME_DATE_RE = re.compile(
    r"_(\d{4})-(\d{1,2})-(\d{1,2})-(\d{1,2})-(\d{1,2})-(\d{1,2})"
)

def wall_clock_start(ulg_path: Path) -> int:
    """Return Unix timestamp in microseconds for the start of this log.

    Priority:
      1. Date embedded in filename  (log_N_YYYY-M-D-H-M-S.ulg)
      2. File modification time
    """
    m = _FILENAME_DATE_RE.search(ulg_path.name)
    if m:
        yr, mo, dy, hr, mi, sc = (int(x) for x in m.groups())
        dt = datetime(yr, mo, dy, hr, mi, sc, tzinfo=timezone.utc)
        return int(dt.timestamp() * 1_000_000)
    return int(ulg_path.stat().st_mtime * 1_000_000)


def wait_for_stable(path: Path, stable_secs: float = 2.0, poll_secs: float = 0.5) -> None:
    """Block until the file size stops changing (i.e. PX4 finished writing)."""
    prev_size = -1
    stable_since = 0.0
    while True:
        try:
            size = path.stat().st_size
        except FileNotFoundError:
            time.sleep(poll_secs)
            continue
        if size == prev_size:
            if stable_since == 0.0:
                stable_since = time.monotonic()
            elif time.monotonic() - stable_since >= stable_secs:
                return
        else:
            prev_size = size
            stable_since = 0.0
        time.sleep(poll_secs)


def ingest_ulg(ulg_path: Path, write_api, org: str) -> int:
    ulog = ULog(str(ulg_path))

    # Anchor to real wall-clock time. PX4 timestamps are µs since boot;
    # we shift them so t=0 aligns with the log's start time on the wall clock.
    t_start_us = wall_clock_start(ulg_path)
    flight_session = datetime.fromtimestamp(
        t_start_us / 1_000_000, tz=timezone.utc
    ).strftime("%Y-%m-%dT%H:%M:%SZ")
    drone = extract_drone_name(ulg_path)

    total_points = 0

    for dataset in ulog.data_list:
        measurement = dataset.name
        data = dataset.data

        if "timestamp" not in data:
            continue

        timestamps = data["timestamp"]  # microseconds since boot, numpy array
        t0_us = int(timestamps[0])      # boot-relative offset of first sample
        field_cols = {
            sanitize_field(k): v
            for k, v in data.items()
            if k != "timestamp" and np.issubdtype(v.dtype, np.number)
        }

        if not field_cols:
            continue

        n = len(timestamps)
        for chunk_start in range(0, n, BATCH_SIZE):
            chunk_end = min(chunk_start + BATCH_SIZE, n)
            records = []
            for i in range(chunk_start, chunk_end):
                fields = {k: float(v[i]) for k, v in field_cols.items()}
                abs_time_ns = (t_start_us + int(timestamps[i]) - t0_us) * 1_000
                records.append({
                    "measurement": measurement,
                    "tags": {
                        "drone": drone,
                        "log_file": ulg_path.name,
                        "flight_session": flight_session,
                    },
                    "fields": fields,
                    "time": abs_time_ns,
                })
            write_api.write(
                bucket=INFLUXDB_BUCKET,
                org=org,
                record=records,
                write_precision="ns",
            )
            total_points += len(records)

        log.debug("  %s: %d points", measurement, n)

    return total_points


def process_file(path: Path, write_api, org: str, state: dict) -> None:
    key = str(path.resolve())
    if key in state:
        return

    log.info("Processing %s …", path.name)
    wait_for_stable(path)

    try:
        points = ingest_ulg(path, write_api, org)
        state[key] = {
            "ingested_at": datetime.now(tz=timezone.utc).isoformat(),
            "points_written": points,
            "log_file": path.name,
        }
        save_state(state)
        log.info("  Done — %d points written", points)
    except Exception as exc:
        log.error("  Failed to ingest %s: %s", path.name, exc)


def backfill(write_api, org: str, state: dict) -> None:
    ulg_files = sorted(WATCH_DIR.rglob("*.ulg"))
    pending = [f for f in ulg_files if str(f.resolve()) not in state]
    if not pending:
        log.info("Backfill: nothing new (all %d files already ingested)", len(ulg_files))
        return
    log.info("Backfill: %d/%d files to ingest", len(pending), len(ulg_files))
    for i, f in enumerate(pending, 1):
        log.info("[%d/%d] %s", i, len(pending), f.relative_to(WATCH_DIR))
        process_file(f, write_api, org, state)

# ---------------------------------------------------------------------------
# Watchdog handler
# ---------------------------------------------------------------------------

class ULGHandler(FileSystemEventHandler):
    def __init__(self, write_api, org: str, state: dict) -> None:
        self._write_api = write_api
        self._org = org
        self._state = state

    def on_created(self, event) -> None:
        src: str = (
            event.src_path.decode() if isinstance(event.src_path, bytes) else event.src_path
        )
        if not event.is_directory and src.endswith(".ulg"):
            log.info("New file detected: %s", src)
            process_file(Path(src), self._write_api, self._org, self._state)

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    if not INFLUXDB_TOKEN:
        log.error("INFLUXDB_TOKEN is not set — check your .env file")
        sys.exit(1)

    if not WATCH_DIR.is_dir():
        log.error("Watch directory not found: %s", WATCH_DIR)
        sys.exit(1)

    log.info("Connecting to InfluxDB at %s …", INFLUXDB_URL)
    client = InfluxDBClient(url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG)

    try:
        client.ping()
    except Exception as exc:
        log.error("Cannot reach InfluxDB: %s", exc)
        sys.exit(1)

    ensure_bucket(client)
    write_api = client.write_api(write_options=SYNCHRONOUS)
    state = load_state()

    log.info("Watch directory: %s", WATCH_DIR)
    backfill(write_api, INFLUXDB_ORG, state)

    handler = ULGHandler(write_api, INFLUXDB_ORG, state)
    observer = Observer()
    observer.schedule(handler, str(WATCH_DIR), recursive=True)
    observer.start()
    log.info("Watching for new .ulg files … (Ctrl+C to stop)")

    try:
        while observer.is_alive():
            observer.join(timeout=1.0)
    except KeyboardInterrupt:
        log.info("Shutting down …")
    finally:
        observer.stop()
        observer.join()
        client.close()
        log.info("Done.")


if __name__ == "__main__":
    main()
