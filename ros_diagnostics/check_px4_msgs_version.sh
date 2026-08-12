#!/bin/bash
# Verifies the running drone_diagnostics container's checked-out px4_msgs
# commit still matches px4_msgs.pin (the single source of truth). Run this
# after every deploy, and especially after a PX4 firmware update on the
# drone, to catch a stale/mismatched build BEFORE trusting its data — a
# mismatch makes the most critical checks (position, EKF, failsafe) fail
# silently via CDR deserialization errors instead of loudly.
#
# Usage: ./check_px4_msgs_version.sh [container_name]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER="${1:-ulg_analysis-drone_diagnostics-1}"
PIN_FILE="$SCRIPT_DIR/px4_msgs.pin"

if [[ ! -f "$PIN_FILE" ]]; then
    echo "[check_px4_msgs_version] FALHOU: $PIN_FILE nao encontrado." >&2
    exit 2
fi

# shellcheck disable=SC1090
source "$PIN_FILE"

if [[ -z "${PX4_MSGS_COMMIT:-}" ]]; then
    echo "[check_px4_msgs_version] FALHOU: PX4_MSGS_COMMIT nao definido em $PIN_FILE." >&2
    exit 2
fi

if ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
    echo "[check_px4_msgs_version] FALHOU: container '$CONTAINER' nao encontrado." >&2
    echo "  (rodando fora do Docker? nao ha como verificar automaticamente — " >&2
    echo "   confira manualmente: git -C <workspace>/src/px4_msgs rev-parse HEAD)" >&2
    exit 2
fi

RUNNING_COMMIT="$(docker exec "$CONTAINER" cat /ros2_ws/src/px4_msgs/.pinned_commit 2>/dev/null || echo "")"

if [[ -z "$RUNNING_COMMIT" ]]; then
    echo "[check_px4_msgs_version] FALHOU: nao foi possivel ler o commit do container" \
         "'$CONTAINER' (imagem antiga sem .pinned_commit? rebuilde a imagem)." >&2
    exit 1
fi

echo "[check_px4_msgs_version] pin esperado ($PIN_FILE): $PX4_MSGS_COMMIT ($PX4_FIRMWARE_VERSION)"
echo "[check_px4_msgs_version] commit no container '$CONTAINER':    $RUNNING_COMMIT"

if [[ "$RUNNING_COMMIT" != "$PX4_MSGS_COMMIT" ]]; then
    echo "[check_px4_msgs_version] FALHOU: mismatch! O container esta rodando um px4_msgs" >&2
    echo "  diferente do pin atual. Rebuilde: cd ../ulg_analysis && docker compose up -d --build drone_diagnostics" >&2
    exit 1
fi

echo "[check_px4_msgs_version] OK: px4_msgs do container bate com o pin."
