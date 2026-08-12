#!/usr/bin/env python3
"""Alerta minimo para o pipeline de diagnostics/logging.

Sem isso, um outage do InfluxDB (gap #1/#2) ou um check em ERROR sustentado
(ex.: falha real de EKF/bateria em voo) so aparecem se alguem abrir o
rqt_robot_monitor ou rodar uma query manual — ninguem e avisado. Este
script roda UMA passada (feito para cron) e dispara um webhook quando:

  (a) nenhum ponto novo foi escrito no InfluxDB nos ultimos N minutos
      (pipeline de logging parado — silenciosamente, do jeito que ja
      aconteceu nesta sessao); ou
  (b) um check ficou em ERROR de forma sustentada nas ultimas N amostras.

"PX4: Time Sync" e ignorado por padrao quando SITL_MODE=1 (ruido conhecido
em simulacao — ver README.md, secao "PX4: Time Sync — SITL vs. real
hardware"). Em hardware real (SITL_MODE=0, o padrao), ele NAO e ignorado,
porque la um ERROR sustentado e sinal genuino de relogio dessincronizado.

Uso (cron a cada 5 min, por exemplo):
    */5 * * * * cd /path/to/ulg_analysis && python3 alert_checker.py >> alert_checker.log 2>&1

Config via env vars (ou .env na pasta pai):
    WEBHOOK_URL              - URL de webhook (Slack-compatible: {"text": "..."})
                                se vazio, so loga no stdout (dry-run).
    NO_DATA_THRESHOLD_MIN    - default 5
    ERROR_SUSTAINED_SAMPLES  - default 10 (checks publicam ~1-2Hz agregados,
                                10 amostras ~= 5-10s sustentado)
    SITL_MODE                - "1" para ignorar ruido conhecido de SITL
                                (default "0" — hardware real assumido)
    IGNORE_CHECKS_SITL       - lista separada por virgula, default
                                "PX4: Time Sync"
"""
import json
import os
import subprocess
import sys
import time
import urllib.request
from collections import defaultdict

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ENV_FILE = os.path.join(SCRIPT_DIR, "..", ".env")
STATE_FILE = os.path.join(SCRIPT_DIR, "alert_checker_state.json")

BUCKET = "drone_diagnostics"
ORG = "aerobatics"
INFLUX_CONTAINER = "ulg_analysis-influxdb-1"


def read_env():
    env = {}
    if os.path.exists(ENV_FILE):
        for line in open(ENV_FILE):
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            k, v = line.split("=", 1)
            env[k] = v
    return env


def load_state():
    if os.path.exists(STATE_FILE):
        try:
            return json.load(open(STATE_FILE))
        except Exception:
            return {}
    return {}


def save_state(state):
    with open(STATE_FILE, "w") as f:
        json.dump(state, f, indent=2)


def influx_query(flux, token):
    cmd = [
        "docker", "exec", INFLUX_CONTAINER,
        "influx", "query", flux,
        "--org", ORG, "--token", token, "--raw",
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    if result.returncode != 0:
        raise RuntimeError(f"influx query falhou: {result.stderr}")
    return result.stdout


def parse_csv_rows(raw):
    rows = []
    header = None
    for line in raw.splitlines():
        if not line or line.startswith("#"):
            continue
        parts = line.split(",")
        if header is None:
            header = parts
            continue
        if len(parts) != len(header):
            continue
        rows.append(dict(zip(header, parts)))
    return rows


def send_webhook(url, text):
    if not url:
        print(f"[alert_checker] (dry-run, WEBHOOK_URL vazio) {text}")
        return
    payload = json.dumps({"text": text}).encode()
    req = urllib.request.Request(url, data=payload, headers={"Content-Type": "application/json"})
    try:
        urllib.request.urlopen(req, timeout=10)
    except Exception as exc:
        print(f"[alert_checker] falha ao enviar webhook: {exc}", file=sys.stderr)


def main():
    env = read_env()
    webhook_url = os.environ.get("WEBHOOK_URL", env.get("ALERT_WEBHOOK_URL", ""))
    no_data_min = int(float(os.environ.get("NO_DATA_THRESHOLD_MIN", "5")))
    error_samples = int(os.environ.get("ERROR_SUSTAINED_SAMPLES", "10"))
    sitl_mode = os.environ.get("SITL_MODE", "0") == "1"
    ignore_checks = set(
        c.strip() for c in os.environ.get("IGNORE_CHECKS_SITL", "PX4: Time Sync").split(",")
    ) if sitl_mode else set()

    token = env.get("INFLUXDB_TOKEN", "")
    if not token:
        print("[alert_checker] INFLUXDB_TOKEN nao encontrado em .env", file=sys.stderr)
        sys.exit(2)

    state = load_state()
    alerts = []

    # (a) pipeline parado: nenhum ponto novo recentemente
    flux_recent = f'''
from(bucket: "{BUCKET}")
  |> range(start: -{no_data_min}m)
  |> filter(fn: (r) => r._field == "level_numeric")
  |> limit(n: 1)
'''
    try:
        recent = parse_csv_rows(influx_query(flux_recent, token))
    except Exception as exc:
        alerts.append(f":rotating_light: alert_checker nao conseguiu consultar o InfluxDB: {exc}")
        recent = None

    if recent is not None and not recent:
        key = "no_data"
        if not state.get(key):
            alerts.append(
                f":rotating_light: Nenhum ponto novo escrito no InfluxDB nos ultimos "
                f"{no_data_min:.0f} min — pipeline de logging pode estar parado "
                f"(InfluxDB fora do ar? drone_diagnostics caiu?)."
            )
        state[key] = True
    elif recent:
        state["no_data"] = False

    # (b) checks em ERROR sustentado
    if recent:
        flux_levels = f'''
from(bucket: "{BUCKET}")
  |> range(start: -{no_data_min}m)
  |> filter(fn: (r) => r._field == "level_numeric")
  |> keep(columns: ["_time", "check_name", "_value"])
'''
        rows = parse_csv_rows(influx_query(flux_levels, token))
        by_check = defaultdict(list)
        for r in rows:
            by_check[r.get("check_name", "?")].append(r.get("_value"))

        for check, values in by_check.items():
            base_name = check.split("/")[-1] if "/" in check else check
            if base_name in ignore_checks:
                continue
            last_n = values[-error_samples:]
            if len(last_n) >= error_samples and all(v == "2" for v in last_n):
                key = f"error:{check}"
                if not state.get(key):
                    alerts.append(
                        f":warning: '{check}' em ERROR sustentado "
                        f"({error_samples}+ amostras seguidas)."
                    )
                state[key] = True
            else:
                state[f"error:{check}"] = False

    for alert in alerts:
        print(f"[alert_checker] {alert}")
        send_webhook(webhook_url, alert)

    if not alerts:
        print(f"[alert_checker] OK — nada para alertar ({time.strftime('%Y-%m-%d %H:%M:%S')}).")

    save_state(state)


if __name__ == "__main__":
    main()
