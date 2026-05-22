"""Tests for sysid_roll.py — ARX roll axis identification."""
import numpy as np
import pandas as pd
import pytest

ULG_PATH = "px4_logs/pixhawk-6X/dia_13-07/log_1_2024-7-13-15-12-16.ulg"


# ---------------------------------------------------------------------------
# load_and_align
# ---------------------------------------------------------------------------

def test_load_and_align_shape():
    """DataFrame combinado deve ter colunas u e y sem NaNs."""
    from sysid_roll import load_and_align
    df = load_and_align(ULG_PATH)
    assert "u" in df.columns
    assert "y" in df.columns
    assert not df["u"].isna().any(), "NaNs em u após interpolação"
    assert not df["y"].isna().any(), "NaNs em y após interpolação"
    assert len(df) > 100, "DataFrame muito curto"


def test_load_and_align_frequency():
    """Frequência de amostragem deve ser ~250 Hz (period=4ms)."""
    from sysid_roll import load_and_align
    df = load_and_align(ULG_PATH)
    diffs = df.index.to_series().diff().dropna()
    median_ms = diffs.median().total_seconds() * 1000
    assert abs(median_ms - 4.0) < 0.5, f"Period médio {median_ms:.2f}ms ≠ 4ms"


# ---------------------------------------------------------------------------
# zero_phase_filter
# ---------------------------------------------------------------------------

def test_zero_phase_filter_no_delay():
    """filtfilt não deve introduzir atraso de fase: impulso centrado deve permanecer centrado."""
    from sysid_roll import zero_phase_filter
    n = 500
    signal = np.zeros(n)
    signal[n // 2] = 1.0
    df = pd.DataFrame({"u": signal, "y": signal.copy()})
    filtered = zero_phase_filter(df.copy(), cutoff_hz=30.0, fs=250.0)
    center_u = np.argmax(np.abs(filtered["u"].values))
    center_y = np.argmax(np.abs(filtered["y"].values))
    assert abs(center_u - n // 2) <= 5, f"u deslocado: pico em {center_u}"
    assert abs(center_y - n // 2) <= 5, f"y deslocado: pico em {center_y}"


def test_zero_phase_filter_attenuates_high_freq():
    """Sinal de 60 Hz deve ser fortemente atenuado com cutoff de 10 Hz a 250 Hz.
    Verifica apenas a região central para evitar artefatos de borda do filtfilt.
    """
    from sysid_roll import zero_phase_filter
    fs = 250.0
    t = np.arange(0, 4, 1 / fs)  # 4 s para minimizar efeito de borda relativo
    high_freq = np.sin(2 * np.pi * 60 * t)
    df = pd.DataFrame({"u": high_freq, "y": high_freq.copy()})
    filtered = zero_phase_filter(df.copy(), cutoff_hz=10.0, fs=fs)
    # Ignora 5% das bordas onde filtfilt pode ter artefatos de inicialização
    margin = int(0.05 * len(filtered))
    central = filtered["u"].iloc[margin:-margin]
    assert central.abs().max() < 0.05, f"Sinal de 60 Hz não foi atenuado no centro: max={central.abs().max():.4f}"


# ---------------------------------------------------------------------------
# select_dynamic_segments
# ---------------------------------------------------------------------------

def test_select_dynamic_segments_removes_static():
    """Trecho constante deve ser removido; trecho com variação suficiente deve ser mantido.

    linspace(0, 20, 200) → diff ≈ 0.10/amostra > threshold=0.05  (dinâmico).
    Zeros → diff = 0 < threshold  (estático, descartado).
    """
    from sysid_roll import select_dynamic_segments
    n = 600
    y = np.zeros(n)
    u = np.zeros(n)
    # Rampa íngreme o suficiente para dy/dt > 0.05 por amostra
    y[200:400] = np.linspace(0, 20, 200)
    u[200:400] = np.linspace(0, 10, 200)
    df = pd.DataFrame({"u": u, "y": y})
    result = select_dynamic_segments(df, threshold=0.05, min_len=10)
    assert len(result) > 0, "Nenhum dado dinâmico encontrado"
    assert len(result) < n, "Trecho estático não foi removido"


def test_select_dynamic_segments_keeps_all_if_fully_dynamic():
    """Se todos os dados forem dinâmicos, deve retornar próximo do total."""
    from sysid_roll import select_dynamic_segments
    rng = np.random.default_rng(0)
    n = 500
    y = np.cumsum(rng.normal(0, 0.5, n))
    u = rng.normal(0, 0.3, n)
    df = pd.DataFrame({"u": u, "y": y})
    result = select_dynamic_segments(df, threshold=0.0, min_len=1)
    assert len(result) >= n - 2


# ---------------------------------------------------------------------------
# build_arx_features
# ---------------------------------------------------------------------------

def test_build_arx_features_shape():
    """Matriz X deve ter na+nb colunas; y_target deve ter mesmo comprimento."""
    from sysid_roll import build_arx_features
    n = 100
    df = pd.DataFrame({"u": np.ones(n), "y": np.arange(n, dtype=float)})
    X, y_target = build_arx_features(df, na=2, nb=2)
    assert X.shape[1] == 4, f"Esperado 4 colunas, obtido {X.shape[1]}"
    assert len(X) == len(y_target)
    assert not np.isnan(X).any()


def test_build_arx_features_lag_values():
    """Verifica que a primeira coluna é y(t-1) e a terceira é u(t-1)."""
    from sysid_roll import build_arx_features
    n = 10
    y_vals = np.arange(n, dtype=float)
    u_vals = np.arange(n, dtype=float) * 2
    df = pd.DataFrame({"u": u_vals, "y": y_vals})
    X, y_target = build_arx_features(df, na=2, nb=2)
    # Primeira linha válida é t=2 (max lag = 2)
    # X[0] = [y(1), y(0), u(1), u(0)]
    assert X[0, 0] == y_vals[1], "Primeira coluna deve ser y(t-1)"
    assert X[0, 1] == y_vals[0], "Segunda coluna deve ser y(t-2)"
    assert X[0, 2] == u_vals[1], "Terceira coluna deve ser u(t-1)"
    assert X[0, 3] == u_vals[0], "Quarta coluna deve ser u(t-2)"


# ---------------------------------------------------------------------------
# fit_arx
# ---------------------------------------------------------------------------

def test_fit_arx_recovers_known_system():
    """Para um sistema ARX sintético conhecido, o fit deve recuperar os coeficientes."""
    from sysid_roll import build_arx_features, fit_arx
    rng = np.random.default_rng(42)
    n = 2000
    a1, a2, b1, b2 = 1.5, -0.7, 0.3, 0.1
    u = rng.normal(0, 1, n)
    y = np.zeros(n)
    for t in range(2, n):
        y[t] = a1 * y[t - 1] + a2 * y[t - 2] + b1 * u[t - 1] + b2 * u[t - 2]
    df = pd.DataFrame({"u": u, "y": y})
    X, y_target = build_arx_features(df, na=2, nb=2)
    coefs, r2 = fit_arx(X, y_target)
    assert r2 > 0.99, f"R² esperado > 0.99, obtido {r2:.4f}"
    np.testing.assert_allclose(
        coefs, [a1, a2, b1, b2], atol=0.01,
        err_msg="Coeficientes não recuperados com precisão",
    )
