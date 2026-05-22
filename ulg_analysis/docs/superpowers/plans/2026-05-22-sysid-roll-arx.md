# Sysid Roll ARX Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implementar `sysid_roll.py` — pipeline completo de identificação ARX do eixo de roll a partir de logs `.ulg` da PX4, produzindo coeficientes, função de transferência discreta e gráfico de validação.

**Architecture:** Script modular com 7 funções puras + `main()`. Cada função é testável isoladamente com dados sintéticos ou reais. Nenhuma função tem side-effect exceto `main()` (que salva a figura) e `print_transfer_function()` (que imprime no terminal).

**Tech Stack:** Python 3.13, pyulog, pandas, scipy, scikit-learn, matplotlib, pytest

---

## File Structure

| Path | Responsabilidade |
|---|---|
| `sysid_roll.py` | Script principal — 7 funções + `main()` |
| `tests/test_sysid_roll.py` | Testes unitários com dados sintéticos |
| `artifacts/` | Saída de figuras (já existe) |

---

### Task 1: Scaffold e testes de `load_and_align`

**Files:**
- Create: `sysid_roll.py`
- Create: `tests/test_sysid_roll.py`

- [ ] **Step 1: Criar `tests/test_sysid_roll.py` com o teste de alinhamento**

```python
# tests/test_sysid_roll.py
import numpy as np
import pandas as pd
import pytest

ULG_PATH = "px4_logs/pixhawk-6X/dia_13-07/log_1_2024-7-13-15-12-16.ulg"

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
```

- [ ] **Step 2: Rodar o teste para confirmar falha**

```bash
cd /mnt/316f1a60-da34-40af-9076-4b3ee9fb0ed1/Documents/PMec/Flying/Diagnostic/ulg_analysis
uv run pytest tests/test_sysid_roll.py::test_load_and_align_shape -v
```
Esperado: `ModuleNotFoundError: No module named 'sysid_roll'`

- [ ] **Step 3: Criar `sysid_roll.py` com `load_and_align`**

```python
# sysid_roll.py
"""Roll axis system identification via ARX model from PX4 .ulg logs."""

from pathlib import Path

import numpy as np
import pandas as pd
from pyulog import ULog
from scipy.signal import butter, filtfilt
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
ULG_PATH      = "px4_logs/pixhawk-6X/dia_13-07/log_1_2024-7-13-15-12-16.ulg"
RESAMPLE_FREQ = "4ms"    # 250 Hz
CUTOFF_HZ     = 30.0
FILTER_ORDER  = 4
ACTIVITY_THR  = 0.05     # rad/s² — |dω/dt| mínimo para trecho dinâmico
MIN_SEG_LEN   = 50       # amostras mínimas por segmento dinâmico
NA, NB        = 2, 2     # ordem ARX
TRAIN_RATIO   = 0.7


def load_and_align(ulg_path: str) -> pd.DataFrame:
    """Carrega vehicle_torque_setpoint (u) e vehicle_angular_velocity (y),
    reamostrar para RESAMPLE_FREQ e combina por inner join.

    Returns:
        DataFrame com colunas ['u', 'y'] indexado por DatetimeIndex.
    """
    log = ULog(ulg_path)
    data = {d.name: d for d in log.data_list}

    def to_series(topic_name: str, field: str) -> pd.Series:
        d = data[topic_name]
        ts = pd.to_datetime(d.data["timestamp"], unit="us")
        s = pd.Series(d.data[field].astype(float), index=ts, name=field)
        s = s[~s.index.duplicated(keep="first")]
        return s.resample(RESAMPLE_FREQ).mean().interpolate(method="time")

    u = to_series("vehicle_torque_setpoint", "xyz[0]").rename("u")
    y = to_series("vehicle_angular_velocity", "xyz[0]").rename("y")

    df = pd.concat([u, y], axis=1, join="inner").dropna()
    return df
```

- [ ] **Step 4: Rodar os testes para confirmar que passam**

```bash
uv run pytest tests/test_sysid_roll.py::test_load_and_align_shape tests/test_sysid_roll.py::test_load_and_align_frequency -v
```
Esperado: `2 passed`

- [ ] **Step 5: Commit**

```bash
git add sysid_roll.py tests/test_sysid_roll.py docs/superpowers/plans/2026-05-22-sysid-roll-arx.md docs/superpowers/specs/2026-05-22-sysid-roll-arx-design.md
git commit -m "feat(sysid): scaffold sysid_roll with load_and_align + tests"
```

---

### Task 2: `zero_phase_filter`

**Files:**
- Modify: `sysid_roll.py` — adicionar `zero_phase_filter`
- Modify: `tests/test_sysid_roll.py` — adicionar testes de filtragem

- [ ] **Step 1: Adicionar testes de filtro em `tests/test_sysid_roll.py`**

```python
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
    """Sinal de 100 Hz deve ser atenuado com cutoff de 30 Hz a 250 Hz."""
    from sysid_roll import zero_phase_filter
    fs = 250.0
    t = np.arange(0, 2, 1 / fs)
    high_freq = np.sin(2 * np.pi * 100 * t)
    df = pd.DataFrame({"u": high_freq, "y": high_freq.copy()})
    filtered = zero_phase_filter(df.copy(), cutoff_hz=30.0, fs=fs)
    assert filtered["u"].abs().max() < 0.1, "Sinal de 100 Hz não foi atenuado"
```

- [ ] **Step 2: Confirmar falha**

```bash
uv run pytest tests/test_sysid_roll.py::test_zero_phase_filter_no_delay -v
```
Esperado: `ImportError` ou `AttributeError`

- [ ] **Step 3: Implementar `zero_phase_filter` em `sysid_roll.py`**

Adicionar após `load_and_align`:

```python
def zero_phase_filter(df: pd.DataFrame, cutoff_hz: float = CUTOFF_HZ, fs: float = 1000 / 4) -> pd.DataFrame:
    """Aplica Butterworth passa-baixa de fase zero em colunas u e y.

    Args:
        df: DataFrame com colunas ['u', 'y'].
        cutoff_hz: Frequência de corte em Hz.
        fs: Frequência de amostragem em Hz (default: 250 Hz para RESAMPLE_FREQ="4ms").

    Returns:
        Cópia do DataFrame com u e y filtrados.
    """
    b, a = butter(FILTER_ORDER, cutoff_hz / (fs / 2), btype="low")
    out = df.copy()
    out["u"] = filtfilt(b, a, df["u"].values)
    out["y"] = filtfilt(b, a, df["y"].values)
    return out
```

- [ ] **Step 4: Rodar testes**

```bash
uv run pytest tests/test_sysid_roll.py::test_zero_phase_filter_no_delay tests/test_sysid_roll.py::test_zero_phase_filter_attenuates_high_freq -v
```
Esperado: `2 passed`

- [ ] **Step 5: Commit**

```bash
git add sysid_roll.py tests/test_sysid_roll.py
git commit -m "feat(sysid): add zero_phase_filter with filtfilt"
```

---

### Task 3: `select_dynamic_segments`

**Files:**
- Modify: `sysid_roll.py` — adicionar `select_dynamic_segments`
- Modify: `tests/test_sysid_roll.py` — adicionar testes de seleção

- [ ] **Step 1: Adicionar testes em `tests/test_sysid_roll.py`**

```python
def test_select_dynamic_segments_removes_static():
    """Trecho constante deve ser removido; trecho com variação deve ser mantido."""
    from sysid_roll import select_dynamic_segments
    n = 600
    y = np.zeros(n)
    u = np.zeros(n)
    # Inserir rampa dinâmica no meio (amostras 200-400)
    y[200:400] = np.linspace(0, 1, 200)
    u[200:400] = np.linspace(0, 0.5, 200)
    df = pd.DataFrame({"u": u, "y": y})
    result = select_dynamic_segments(df, threshold=0.05, min_len=10)
    assert len(result) > 0, "Nenhum dado dinâmico encontrado"
    assert len(result) < n, "Trecho estático não foi removido"


def test_select_dynamic_segments_keeps_all_if_fully_dynamic():
    """Se todos os dados forem dinâmicos, deve retornar próximo do total (menos bordas de lags)."""
    from sysid_roll import select_dynamic_segments
    rng = np.random.default_rng(0)
    n = 500
    y = np.cumsum(rng.normal(0, 0.5, n))
    u = rng.normal(0, 0.3, n)
    df = pd.DataFrame({"u": u, "y": y})
    result = select_dynamic_segments(df, threshold=0.0, min_len=1)
    assert len(result) >= n - 2
```

- [ ] **Step 2: Confirmar falha**

```bash
uv run pytest tests/test_sysid_roll.py::test_select_dynamic_segments_removes_static -v
```
Esperado: `ImportError` ou `AttributeError`

- [ ] **Step 3: Implementar `select_dynamic_segments` em `sysid_roll.py`**

Adicionar após `zero_phase_filter`:

```python
def select_dynamic_segments(
    df: pd.DataFrame,
    threshold: float = ACTIVITY_THR,
    min_len: int = MIN_SEG_LEN,
) -> pd.DataFrame:
    """Retém apenas trechos onde |dy/dt| > threshold, descartando hover estático.

    A derivada é calculada por diferença finita. Segmentos contíguos abaixo
    do limiar e mais curtos que min_len amostras são descartados.

    Args:
        df: DataFrame com colunas ['u', 'y'].
        threshold: Limiar mínimo de |dy/dt| em rad/s por amostra.
        min_len: Comprimento mínimo de um segmento ativo (amostras).

    Returns:
        DataFrame filtrado (pode ter índice não contíguo).
    """
    dy_dt = df["y"].diff().abs().fillna(0.0)
    active = dy_dt > threshold

    # Expande a máscara: mantém apenas segmentos contíguos com >= min_len pontos
    mask = active.copy()
    in_seg = False
    seg_start = 0
    indices = list(range(len(df)))
    drop_indices: list[int] = []

    for i in indices:
        if active.iloc[i]:
            if not in_seg:
                seg_start = i
                in_seg = True
        else:
            if in_seg:
                if i - seg_start < min_len:
                    drop_indices.extend(range(seg_start, i))
                in_seg = False
                mask.iloc[seg_start:i] = i - seg_start >= min_len
    if in_seg:
        seg_len = len(df) - seg_start
        if seg_len < min_len:
            mask.iloc[seg_start:] = False

    return df[mask]
```

- [ ] **Step 4: Rodar testes**

```bash
uv run pytest tests/test_sysid_roll.py::test_select_dynamic_segments_removes_static tests/test_sysid_roll.py::test_select_dynamic_segments_keeps_all_if_fully_dynamic -v
```
Esperado: `2 passed`

- [ ] **Step 5: Commit**

```bash
git add sysid_roll.py tests/test_sysid_roll.py
git commit -m "feat(sysid): add select_dynamic_segments"
```

---

### Task 4: `build_arx_features`

**Files:**
- Modify: `sysid_roll.py` — adicionar `build_arx_features`
- Modify: `tests/test_sysid_roll.py` — adicionar testes de construção de features

- [ ] **Step 1: Adicionar testes em `tests/test_sysid_roll.py`**

```python
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
```

- [ ] **Step 2: Confirmar falha**

```bash
uv run pytest tests/test_sysid_roll.py::test_build_arx_features_shape -v
```
Esperado: `ImportError` ou `AttributeError`

- [ ] **Step 3: Implementar `build_arx_features` em `sysid_roll.py`**

Adicionar após `select_dynamic_segments`:

```python
def build_arx_features(
    df: pd.DataFrame,
    na: int = NA,
    nb: int = NB,
) -> tuple[np.ndarray, np.ndarray]:
    """Monta a matriz de regressão ARX com lags de y e u.

    Modelo: y(t) = a1*y(t-1) + ... + ana*y(t-na) + b1*u(t-1) + ... + bnb*u(t-nb)

    Args:
        df: DataFrame com colunas ['u', 'y'] (índice contíguo após reset).
        na: Número de lags da saída.
        nb: Número de lags da entrada (atraso mínimo = 1 amostra).

    Returns:
        X: np.ndarray shape (N - max_lag, na + nb)
        y_target: np.ndarray shape (N - max_lag,)
    """
    df = df.reset_index(drop=True)
    y = df["y"].values
    u = df["u"].values
    max_lag = max(na, nb)
    n = len(df) - max_lag

    cols = []
    for i in range(1, na + 1):
        cols.append(y[max_lag - i : max_lag - i + n])
    for i in range(1, nb + 1):
        cols.append(u[max_lag - i : max_lag - i + n])

    X = np.column_stack(cols)
    y_target = y[max_lag : max_lag + n]
    return X, y_target
```

- [ ] **Step 4: Rodar testes**

```bash
uv run pytest tests/test_sysid_roll.py::test_build_arx_features_shape tests/test_sysid_roll.py::test_build_arx_features_lag_values -v
```
Esperado: `2 passed`

- [ ] **Step 5: Commit**

```bash
git add sysid_roll.py tests/test_sysid_roll.py
git commit -m "feat(sysid): add build_arx_features with lag matrix"
```

---

### Task 5: `fit_arx` e `print_transfer_function`

**Files:**
- Modify: `sysid_roll.py` — adicionar `fit_arx` e `print_transfer_function`
- Modify: `tests/test_sysid_roll.py` — adicionar teste de fitting

- [ ] **Step 1: Adicionar teste em `tests/test_sysid_roll.py`**

```python
def test_fit_arx_recovers_known_system():
    """Para um sistema ARX sintético conhecido, o fit deve recuperar os coeficientes."""
    from sysid_roll import build_arx_features, fit_arx
    rng = np.random.default_rng(42)
    n = 2000
    a1, a2, b1, b2 = 1.5, -0.7, 0.3, 0.1
    u = rng.normal(0, 1, n)
    y = np.zeros(n)
    for t in range(2, n):
        y[t] = a1 * y[t-1] + a2 * y[t-2] + b1 * u[t-1] + b2 * u[t-2]
    df = pd.DataFrame({"u": u, "y": y})
    X, y_target = build_arx_features(df, na=2, nb=2)
    coefs, r2 = fit_arx(X, y_target)
    assert r2 > 0.99, f"R² esperado > 0.99, obtido {r2:.4f}"
    np.testing.assert_allclose(coefs, [a1, a2, b1, b2], atol=0.01,
                               err_msg="Coeficientes não recuperados com precisão")
```

- [ ] **Step 2: Confirmar falha**

```bash
uv run pytest tests/test_sysid_roll.py::test_fit_arx_recovers_known_system -v
```
Esperado: `ImportError` ou `AttributeError`

- [ ] **Step 3: Implementar `fit_arx` e `print_transfer_function` em `sysid_roll.py`**

Adicionar após `build_arx_features`:

```python
def fit_arx(
    X_train: np.ndarray,
    y_train: np.ndarray,
) -> tuple[np.ndarray, float]:
    """Ajusta modelo ARX por regressão linear (OLS).

    Args:
        X_train: Matriz de features shape (N, na+nb).
        y_train: Vetor alvo shape (N,).

    Returns:
        coefs: np.ndarray [a1..ana, b1..bnb]
        r2: Coeficiente de determinação no treino.
    """
    model = LinearRegression(fit_intercept=False)
    model.fit(X_train, y_train)
    r2 = r2_score(y_train, model.predict(X_train))
    return model.coef_, r2


def print_transfer_function(coefs: np.ndarray, na: int, nb: int, dt: float) -> None:
    """Exibe H(z) = B(z)/A(z) com polos e zeros.

    H(z) = (b1*z^(na-1) + b2*z^(na-2) + ...) / (z^na - a1*z^(na-1) - a2*z^(na-2) - ...)

    Args:
        coefs: [a1..ana, b1..bnb] retornado por fit_arx.
        na: Número de lags da saída.
        nb: Número de lags da entrada.
        dt: Período de amostragem em segundos.
    """
    a_coefs = coefs[:na]
    b_coefs = coefs[na:]

    # Polinômios em z^-1 → converter para coeficientes em z (ordem decrescente)
    # A(z) = z^na - a1*z^(na-1) - a2*z^(na-2) - ...
    A = np.concatenate([[1.0], -a_coefs])
    # B(z) = b1*z^(na-1) + b2*z^(na-2) + ... (numerador alinhado ao denominador)
    B = np.concatenate([np.zeros(na - nb), b_coefs])

    poles = np.roots(A)
    zeros = np.roots(B)
    dc_gain = np.polyval(B, 1.0) / np.polyval(A, 1.0)

    print("\n" + "=" * 60)
    print("FUNÇÃO DE TRANSFERÊNCIA DISCRETA  H(z) = B(z)/A(z)")
    print(f"Período de amostragem: dt = {dt*1000:.1f} ms  (fs = {1/dt:.0f} Hz)")
    print("=" * 60)
    print(f"Coeficientes ARX:  a = {a_coefs},  b = {b_coefs}")
    print(f"\nA(z) = {' - '.join([f'{c:.6f}·z^{na-i}' for i, c in enumerate(a_coefs, 1)])}")
    print(f"     = z^{na} - {' - '.join([f'{c:.6f}·z^{na-i}' for i, c in enumerate(a_coefs, 1)])}")
    print(f"\nB(z) = {' + '.join([f'{c:.6f}·z^{na-i}' for i, c in enumerate(b_coefs, 1)])}")
    print(f"\nPolos:  {[f'{p:.4f}' for p in poles]}")
    print(f"Zeros:  {[f'{z:.4f}' for z in zeros]}")
    print(f"Ganho DC (z=1):  {dc_gain:.6f}  rad/s por Nm")
    print("=" * 60 + "\n")
```

- [ ] **Step 4: Rodar testes**

```bash
uv run pytest tests/test_sysid_roll.py::test_fit_arx_recovers_known_system -v
```
Esperado: `1 passed`

- [ ] **Step 5: Commit**

```bash
git add sysid_roll.py tests/test_sysid_roll.py
git commit -m "feat(sysid): add fit_arx OLS and print_transfer_function"
```

---

### Task 6: `validate` e `main`

**Files:**
- Modify: `sysid_roll.py` — adicionar `validate` e `main`

Não há teste unitário para `validate` (depende de matplotlib) nem para `main` (end-to-end). O próprio output de `main` é a validação visual.

- [ ] **Step 1: Implementar `validate` em `sysid_roll.py`**

Adicionar após `print_transfer_function`:

```python
def validate(
    model_coefs: np.ndarray,
    X_test: np.ndarray,
    y_test: np.ndarray,
    na: int = NA,
    nb: int = NB,
    dt: float = 4e-3,
    save_path: str | None = None,
) -> float:
    """Gera gráfico de validação: roll rate real vs predito + erro + espectro do resíduo.

    Args:
        model_coefs: Coeficientes ARX [a1..ana, b1..bnb].
        X_test: Matriz de features do conjunto de teste.
        y_test: Vetor de roll rate real (conjunto de teste).
        na: Ordem ARX da saída.
        nb: Ordem ARX da entrada.
        dt: Período de amostragem em segundos.
        save_path: Caminho para salvar a figura. Se None, exibe interativamente.

    Returns:
        r2_test: R² no conjunto de teste.
    """
    model = LinearRegression(fit_intercept=False)
    model.coef_ = model_coefs
    model.intercept_ = 0.0

    y_pred = model.predict(X_test)
    r2_test = r2_score(y_test, y_pred)
    residual = y_test - y_pred
    t = np.arange(len(y_test)) * dt

    fig, axes = plt.subplots(3, 1, figsize=(12, 9))
    fig.suptitle(f"ARX Roll Sysid — Validação  (R² = {r2_test:.4f})", fontsize=13)

    axes[0].plot(t, y_test, label="Roll rate real", alpha=0.8)
    axes[0].plot(t, y_pred, label="ARX predito", linestyle="--", alpha=0.8)
    axes[0].set_ylabel("Roll rate (rad/s)")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(t, residual, color="tab:red", alpha=0.7)
    axes[1].set_ylabel("Erro (rad/s)")
    axes[1].set_xlabel("Tempo (s)")
    axes[1].grid(True)

    freqs = np.fft.rfftfreq(len(residual), d=dt)
    psd = np.abs(np.fft.rfft(residual)) ** 2
    axes[2].semilogy(freqs, psd, color="tab:purple", alpha=0.7)
    axes[2].set_xlabel("Frequência (Hz)")
    axes[2].set_ylabel("PSD resíduo")
    axes[2].set_xlim(0, 1 / dt / 2)
    axes[2].grid(True)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Figura salva em: {save_path}")
    else:
        plt.show()
    plt.close()
    return r2_test
```

- [ ] **Step 2: Implementar `main` em `sysid_roll.py`**

Adicionar ao final do arquivo:

```python
def main(ulg_path: str = ULG_PATH) -> None:
    """Pipeline completo de identificação ARX do eixo de roll."""
    print(f"Carregando {ulg_path} ...")
    df = load_and_align(ulg_path)
    print(f"  {len(df)} amostras após resample/align")

    print("Filtrando (Butterworth fase zero) ...")
    fs = 1000.0 / int(RESAMPLE_FREQ.replace("ms", ""))
    df = zero_phase_filter(df, cutoff_hz=CUTOFF_HZ, fs=fs)

    print(f"Selecionando segmentos dinâmicos (threshold={ACTIVITY_THR} rad/s²) ...")
    df = select_dynamic_segments(df, threshold=ACTIVITY_THR, min_len=MIN_SEG_LEN)
    print(f"  {len(df)} amostras dinâmicas retidas")

    print(f"Construindo features ARX (na={NA}, nb={NB}) ...")
    X, y_target = build_arx_features(df, na=NA, nb=NB)

    split = int(len(X) * TRAIN_RATIO)
    X_train, X_test = X[:split], X[split:]
    y_train, y_test = y_target[:split], y_target[split:]
    print(f"  Treino: {len(X_train)} amostras  |  Teste: {len(X_test)} amostras")

    print("Ajustando modelo ARX ...")
    coefs, r2_train = fit_arx(X_train, y_train)
    print(f"  R² treino = {r2_train:.4f}")

    print_transfer_function(coefs, na=NA, nb=NB, dt=4e-3)

    stem = Path(ulg_path).stem
    save_path = f"artifacts/sysid_roll_{stem}.png"
    r2_test = validate(coefs, X_test, y_test, na=NA, nb=NB, dt=4e-3, save_path=save_path)
    print(f"R² validação = {r2_test:.4f}")


if __name__ == "__main__":
    import sys
    path = sys.argv[1] if len(sys.argv) > 1 else ULG_PATH
    main(path)
```

- [ ] **Step 3: Rodar a suite completa**

```bash
uv run pytest tests/test_sysid_roll.py -v
```
Esperado: todos os testes passam

- [ ] **Step 4: Rodar o pipeline de ponta a ponta**

```bash
uv run python sysid_roll.py
```
Esperado: output no terminal com coeficientes ARX, H(z), R² de treino e validação, e figura salva em `artifacts/`.

- [ ] **Step 5: Commit final**

```bash
git add sysid_roll.py
git commit -m "feat(sysid): add validate and main — complete ARX sysid pipeline"
```
