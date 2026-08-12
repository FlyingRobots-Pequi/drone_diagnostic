#!/bin/bash
# Periodic InfluxDB backup — exports the drone_diagnostics + px4_flight_logs
# data outside the Docker volume. Without this, `docker compose down -v`,
# disk corruption, or an accidental volume removal loses ALL diagnostics
# history and ingested .ulg telemetry with no way to recover it (see
# docs/production_readiness_plan.md gap #7).
#
# Usage:
#   ./backup_influxdb.sh [destination_dir]
#
# destination_dir defaults to $BACKUP_DEST_DIR or ./backups (should be
# pointed at storage OUTSIDE this host for real protection — a mounted
# NAS/S3-backed path/external disk; a local-only destination only protects
# against volume corruption, not host loss).
#
# Schedule with cron, e.g. daily at 03:00:
#   0 3 * * * cd /path/to/Flying/Diagnostic/ulg_analysis && ./backup_influxdb.sh >> backup.log 2>&1
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST_DIR="${1:-${BACKUP_DEST_DIR:-$SCRIPT_DIR/backups}}"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
BACKUP_PATH="$DEST_DIR/influxdb-backup-$STAMP"

# Load INFLUXDB_BACKUP_TOKEN from the shared .env without leaking it into
# shell history / process listing of other commands. This must be a token
# with OPERATOR scope (not the app's org-scoped INFLUXDB_TOKEN) — `influx
# backup` downloads an instance-wide metadata snapshot that an org-scoped
# all-access token is not permitted to read. See README section on
# "regenerating the backup token" if this one is ever lost/rotated: use
# `influxd recovery auth create-operator` (stop the container first — it
# needs exclusive access to the bolt.db file).
ENV_FILE="$SCRIPT_DIR/../.env"
INFLUXDB_TOKEN="$(python3 -c "
import sys
for line in open('$ENV_FILE'):
    if line.startswith('INFLUXDB_BACKUP_TOKEN='):
        print(line.strip().split('=', 1)[1])
        sys.exit(0)
sys.exit(1)
")"

mkdir -p "$DEST_DIR"

echo "[backup_influxdb] backing up to container path /tmp/backup, then copying to $BACKUP_PATH ..."
docker exec ulg_analysis-influxdb-1 influx backup /tmp/backup \
    --org aerobatics --token "$INFLUXDB_TOKEN"
docker cp ulg_analysis-influxdb-1:/tmp/backup "$BACKUP_PATH"
docker exec ulg_analysis-influxdb-1 rm -rf /tmp/backup

echo "[backup_influxdb] done: $BACKUP_PATH"

# Keep only the last N backups locally (older ones should already be
# synced to off-host storage by whatever schedules this script, e.g. an
# rsync/rclone step after this one, or DEST_DIR itself being a mounted
# remote path).
KEEP="${BACKUP_KEEP_COUNT:-7}"
ls -1dt "$DEST_DIR"/influxdb-backup-* 2>/dev/null | tail -n +$((KEEP + 1)) | while read -r old; do
    echo "[backup_influxdb] pruning old local backup: $old"
    rm -rf "$old"
done
