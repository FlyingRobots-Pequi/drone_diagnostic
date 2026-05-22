"""Roll axis system identification via ARX model from PX4 .ulg logs.

Pipeline:
  1. load_and_align      — extrai u (torque) e y (roll rate) do .ulg, resample 250 Hz
  2. zero_phase_filter   — Butterworth passa-baixa de fase zero em u e y
  3. select_dynamic_segments — descarta hover estático, retém trechos dinâmicos
  4. build_arx_features  — monta matriz de regressão com lags de y e u
  5. fit_arx             — OLS → coeficientes [a1, a2, b1, b2]
  6. print_transfer_function — exibe H(z) = B(z)/A(z), polos, zeros, ganho DC
  7. validate            — R² no conjunto de teste + 3 gráficos de validação
  8. main                — orquestra tudo, salva figura em artifacts/

Modelo ARX(na=2, nb=2):
    y(t) = a1·y(t-1) + a2·y(t-2) + b1·u(t-1) + b2·u(t-2)

Função de Transferência discreta:
    H(z) = Y(z)/U(z) = (b1·z + b2) / (z² - a1·z - a2)

Sinais:
    u(t)  ← vehicle_torque_setpoint.xyz[0]  [Nm]
    y(t)  ← vehicle_angular_velocity.xyz[0] [rad/s]
"""

from pathlib import Path
import sys

import numpy as np
import pandas as pd
from pyulog import ULog
from scipy.signal import butter, filtfilt
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
ULG_PATH      = "px4_logs/pixhawk-6X/dia_13-07/log_1_2024-7-13-15-12-16.ulg"
RESAMPLE_FREQ = "4ms"    # 250 Hz
CUTOFF_HZ     = 30.0     # Butterworth LP cutoff (Hz)
FILTER_ORDER  = 4
ACTIVITY_THR  = 0.004    # |dω/dt| mínimo (rad/s por amostra) para trecho dinâmico
MIN_SEG_LEN   = 20       # amostras mínimas por segmento dinâmico
NA, NB        = 2, 2     # ordem ARX (lags de y e u)
TRAIN_RATIO   = 0.7      # fração temporal para treino


# ---------------------------------------------------------------------------
# 1. Carregamento e alinhamento temporal
# ---------------------------------------------------------------------------

def load_and_align(ulg_path: str) -> pd.DataFrame:
    """Carrega vehicle_torque_setpoint (u) e vehicle_angular_velocity (y),
    reamostrar independentemente para RESAMPLE_FREQ e combina por inner join.

    Cada tópico é reamostrado isoladamente antes da combinação para garantir
    alinhamento temporal correto sem introduzir NaNs cruzados.

    Returns:
        DataFrame com colunas ['u', 'y'] indexado por DatetimeIndex uniforme.
    """
    log = ULog(ulg_path)
    data = {d.name: d for d in log.data_list}

    def to_series(topic_name: str, field: str, label: str) -> pd.Series:
        d = data[topic_name]
        ts = pd.to_datetime(d.data["timestamp"], unit="us")
        s = pd.Series(d.data[field].astype(float), index=ts, name=label)
        # Remove duplicatas de timestamp (podem ocorrer em multi-instance topics)
        s = s[~s.index.duplicated(keep="first")]
        return s.resample(RESAMPLE_FREQ).mean().interpolate(method="time")

    u = to_series("vehicle_torque_setpoint", "xyz[0]", "u")
    y = to_series("vehicle_angular_velocity", "xyz[0]", "y")

    df = pd.concat([u, y], axis=1, join="inner").dropna()
    return df


# ---------------------------------------------------------------------------
# 2. Filtragem de fase zero
# ---------------------------------------------------------------------------

def zero_phase_filter(
    df: pd.DataFrame,
    cutoff_hz: float = CUTOFF_HZ,
    fs: float = 1000.0 / 4,  # 250 Hz para RESAMPLE_FREQ="4ms"
) -> pd.DataFrame:
    """Aplica Butterworth passa-baixa de fase zero (filtfilt) em u e y.

    filtfilt processa o sinal nos dois sentidos, cancelando o atraso de grupo
    do filtro. Isso é essencial: um filtro causal introduziria um atraso entre
    u e y que a regressão ARX interpretaria erroneamente como dinâmica da planta.

    Args:
        df: DataFrame com colunas ['u', 'y'].
        cutoff_hz: Frequência de corte em Hz.
        fs: Frequência de amostragem em Hz.

    Returns:
        Cópia do DataFrame com u e y filtrados.
    """
    b, a = butter(FILTER_ORDER, cutoff_hz / (fs / 2.0), btype="low")
    out = df.copy()
    out["u"] = filtfilt(b, a, df["u"].values)
    out["y"] = filtfilt(b, a, df["y"].values)
    return out


# ---------------------------------------------------------------------------
# 3. Seleção de segmentos dinâmicos
# ---------------------------------------------------------------------------

def select_dynamic_segments(
    df: pd.DataFrame,
    threshold: float = ACTIVITY_THR,
    min_len: int = MIN_SEG_LEN,
) -> pd.DataFrame:
    """Retém apenas trechos onde |dy/dt| > threshold, descartando hover estático.

    Em voo pairado (hover) o sistema está em regime permanente: a saída y é
    aproximadamente constante e o controlador compensa perturbações menores.
    Incluir esses trechos na regressão ARX introduz viés pois o modelo passaria
    a capturar o comportamento do PID (loop fechado) em vez da planta.

    A derivada discreta de y é calculada por diferença finita de primeira ordem.
    Segmentos contíguos com |dy/dt| > threshold e comprimento >= min_len são
    mantidos; os demais são descartados.

    Args:
        df: DataFrame com colunas ['u', 'y'] e índice qualquer.
        threshold: Limiar de |dy/dt| em unidades de rad/s por amostra.
        min_len: Comprimento mínimo de segmento ativo em amostras.

    Returns:
        DataFrame filtrado (índice original preservado, pode ser não contíguo).
    """
    dy_dt = df["y"].diff().abs().fillna(0.0)
    active_mask = (dy_dt > threshold).values

    # Identifica segmentos contíguos ativos e filtra por min_len
    keep = np.zeros(len(df), dtype=bool)
    i = 0
    while i < len(active_mask):
        if active_mask[i]:
            j = i + 1
            while j < len(active_mask) and active_mask[j]:
                j += 1
            if j - i >= min_len:
                keep[i:j] = True
            i = j
        else:
            i += 1

    return df[keep]


# ---------------------------------------------------------------------------
# 4. Construção da matriz ARX
# ---------------------------------------------------------------------------

def build_arx_features(
    df: pd.DataFrame,
    na: int = NA,
    nb: int = NB,
) -> tuple[np.ndarray, np.ndarray]:
    """Monta a matriz de regressão ARX com lags de y e u.

    Modelo:
        y(t) = a1·y(t-1) + ... + ana·y(t-na) + b1·u(t-1) + ... + bnb·u(t-nb)

    A entrada começa em u(t-1) (atraso mínimo de 1 amostra), o que reflete a
    causalidade física: o torque comandado em t afeta o roll rate em t+1.

    Args:
        df: DataFrame com colunas ['u', 'y']. O índice é ignorado (reset interno).
        na: Número de lags da saída y.
        nb: Número de lags da entrada u (atraso mínimo = 1 amostra).

    Returns:
        X:        np.ndarray shape (N - max_lag, na + nb)
                  Colunas: [y(t-1)..y(t-na), u(t-1)..u(t-nb)]
        y_target: np.ndarray shape (N - max_lag,) — valores de y(t)
    """
    df = df.reset_index(drop=True)
    y = df["y"].values
    u = df["u"].values
    max_lag = max(na, nb)
    n_valid = len(df) - max_lag

    cols = []
    for i in range(1, na + 1):
        cols.append(y[max_lag - i : max_lag - i + n_valid])
    for i in range(1, nb + 1):
        cols.append(u[max_lag - i : max_lag - i + n_valid])

    X = np.column_stack(cols)
    y_target = y[max_lag : max_lag + n_valid]
    return X, y_target


# ---------------------------------------------------------------------------
# 5. Fitting ARX por OLS
# ---------------------------------------------------------------------------

def fit_arx(
    X_train: np.ndarray,
    y_train: np.ndarray,
) -> tuple[np.ndarray, float]:
    """Ajusta modelo ARX por regressão linear (OLS, sem intercepto).

    O modelo ARX é linear nos parâmetros: y = X·θ onde θ = [a1..ana, b1..bnb].
    OLS minimiza ‖y - X·θ‖², equivalente ao estimador de máxima verossimilhança
    assumindo ruído gaussiano branco nos resíduos.

    Args:
        X_train: Matriz de features shape (N, na+nb).
        y_train: Vetor alvo shape (N,).

    Returns:
        coefs: np.ndarray [a1..ana, b1..bnb]
        r2:    Coeficiente de determinação no conjunto de treino.
    """
    model = LinearRegression(fit_intercept=False)
    model.fit(X_train, y_train)
    r2 = r2_score(y_train, model.predict(X_train))
    return model.coef_, r2


# ---------------------------------------------------------------------------
# 6. Exibição da função de transferência
# ---------------------------------------------------------------------------

def print_transfer_function(
    coefs: np.ndarray,
    na: int = NA,
    nb: int = NB,
    dt: float = 4e-3,
) -> None:
    """Exibe H(z) = B(z)/A(z) com polos, zeros e ganho DC.

    Conversão dos coeficientes ARX para a forma z:

        A(z) = z^na - a1·z^(na-1) - ... - ana
        B(z) = b1·z^(na-1) + b2·z^(na-2) + ... (alinhado ao grau de A)

        H(z) = B(z) / A(z)

    Para na=2, nb=2:
        H(z) = (b1·z + b2) / (z² - a1·z - a2)

    Args:
        coefs: Array [a1..ana, b1..bnb] retornado por fit_arx.
        na:    Número de lags da saída.
        nb:    Número de lags da entrada.
        dt:    Período de amostragem em segundos.
    """
    a_coefs = coefs[:na]
    b_coefs = coefs[na:]

    # Constrói A(z) = [1, -a1, -a2, ...] e B(z) alinhado ao mesmo grau
    A = np.concatenate([[1.0], -a_coefs])
    B = np.concatenate([np.zeros(na - nb), b_coefs])

    poles = np.roots(A)
    zeros = np.roots(B) if nb > 0 else np.array([])
    dc_gain = np.polyval(B, 1.0) / np.polyval(A, 1.0)

    sep = "=" * 62
    print(f"\n{sep}")
    print("  FUNÇÃO DE TRANSFERÊNCIA DISCRETA  H(z) = B(z) / A(z)")
    print(f"  dt = {dt * 1000:.1f} ms   fs = {1 / dt:.0f} Hz")
    print(sep)
    print(f"  Coeficientes ARX:")
    for i, ai in enumerate(a_coefs, 1):
        print(f"    a{i} = {ai: .6f}")
    for i, bi in enumerate(b_coefs, 1):
        print(f"    b{i} = {bi: .6f}")

    if na == 2 and nb == 2:
        a1, a2 = a_coefs
        b1, b2 = b_coefs
        b_str = f"{b1:.6f}·z + {b2:.6f}" if b2 >= 0 else f"{b1:.6f}·z - {abs(b2):.6f}"
        a_str = f"z² - {a1:.6f}·z + {a2:.6f}" if a2 >= 0 else f"z² - {a1:.6f}·z - {abs(a2):.6f}"
        print(f"\n  H(z) = ({b_str})")
        print(f"         {'─' * (len(b_str) + 2)}")
        print(f"         ({a_str})")

    print(f"\n  Polos: {['  ' + str(round(p.real, 5)) + (f' + {p.imag:.5f}j' if abs(p.imag) > 1e-8 else '') for p in poles]}")
    if len(zeros):
        print(f"  Zeros: {['  ' + str(round(z.real, 5)) + (f' + {z.imag:.5f}j' if abs(z.imag) > 1e-8 else '') for z in zeros]}")
    print(f"\n  Ganho DC (z=1): {dc_gain:.6f}  (rad/s) / Nm")

    # Estabilidade: polos dentro do círculo unitário
    max_pole_mag = max(abs(poles))
    stable = max_pole_mag < 1.0
    print(f"  |polo máximo| = {max_pole_mag:.6f}  →  {'ESTÁVEL ✓' if stable else 'INSTÁVEL ✗'}")
    print(sep + "\n")


# ---------------------------------------------------------------------------
# 7. Validação e plot
# ---------------------------------------------------------------------------

def validate(
    coefs: np.ndarray,
    X_test: np.ndarray,
    y_test: np.ndarray,
    dt: float = 4e-3,
    save_path: str | None = None,
) -> float:
    """Gera gráfico de validação com 3 subplots e retorna R² no conjunto de teste.

    Subplots:
        1. Roll rate real vs predito
        2. Erro de predição ao longo do tempo
        3. Espectro de potência do resíduo (identifica frequências não capturadas)

    Args:
        coefs:     Coeficientes ARX [a1..ana, b1..bnb].
        X_test:    Matriz de features do conjunto de teste.
        y_test:    Roll rate real no conjunto de teste (rad/s).
        dt:        Período de amostragem em segundos.
        save_path: Caminho para salvar figura. None → exibe interativamente.

    Returns:
        r2_test: R² no conjunto de teste.
    """
    y_pred = X_test @ coefs
    r2_test = r2_score(y_test, y_pred)
    residual = y_test - y_pred
    t = np.arange(len(y_test)) * dt

    fig, axes = plt.subplots(3, 1, figsize=(13, 9))
    fig.suptitle(
        f"ARX Roll Sysid — Validação  |  R² treino/teste mostrado abaixo",
        fontsize=12,
    )

    # Subplot 1: real vs predito
    axes[0].plot(t, y_test, label="Roll rate real", alpha=0.85, linewidth=0.9)
    axes[0].plot(t, y_pred, label=f"ARX predito  (R²={r2_test:.4f})",
                 linestyle="--", alpha=0.85, linewidth=0.9)
    axes[0].set_ylabel("Roll rate (rad/s)")
    axes[0].legend(fontsize=9)
    axes[0].grid(True, alpha=0.4)

    # Subplot 2: erro
    axes[1].plot(t, residual, color="tab:red", alpha=0.75, linewidth=0.8)
    axes[1].axhline(0, color="k", linewidth=0.5, linestyle="--")
    axes[1].set_ylabel("Erro (rad/s)")
    axes[1].set_xlabel("Tempo (s)")
    axes[1].grid(True, alpha=0.4)

    # Subplot 3: PSD do resíduo
    freqs = np.fft.rfftfreq(len(residual), d=dt)
    psd = np.abs(np.fft.rfft(residual)) ** 2
    axes[2].semilogy(freqs, psd + 1e-20, color="tab:purple", alpha=0.75, linewidth=0.8)
    axes[2].set_xlabel("Frequência (Hz)")
    axes[2].set_ylabel("PSD resíduo")
    axes[2].set_xlim(0, 0.5 / dt)
    axes[2].grid(True, alpha=0.4)

    plt.tight_layout()
    if save_path:
        Path(save_path).parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Figura salva em: {save_path}")
    else:
        plt.show()
    plt.close()
    return r2_test


# ---------------------------------------------------------------------------
# 8. Pipeline principal
# ---------------------------------------------------------------------------

def main(ulg_path: str = ULG_PATH) -> None:
    """Executa o pipeline completo de sysid ARX do eixo de roll."""
    fs = 1000.0 / int(RESAMPLE_FREQ.replace("ms", ""))
    dt = 1.0 / fs

    print(f"\n[1/6] Carregando: {ulg_path}")
    df = load_and_align(ulg_path)
    print(f"      {len(df)} amostras após resample/align  ({len(df) * dt:.1f} s)")

    print(f"[2/6] Filtragem Butterworth fase zero  (corte={CUTOFF_HZ} Hz, ordem={FILTER_ORDER})")
    df = zero_phase_filter(df, cutoff_hz=CUTOFF_HZ, fs=fs)

    print(f"[3/6] Seleção de segmentos dinâmicos  (threshold={ACTIVITY_THR} rad/s·amostra⁻¹, min={MIN_SEG_LEN} amostras)")
    df_dyn = select_dynamic_segments(df, threshold=ACTIVITY_THR, min_len=MIN_SEG_LEN)
    pct = 100 * len(df_dyn) / len(df)
    print(f"      {len(df_dyn)} amostras dinâmicas retidas  ({pct:.1f}% do total)")

    if len(df_dyn) < 4 * (NA + NB):
        raise RuntimeError(
            f"Dados insuficientes após filtragem ({len(df_dyn)} amostras). "
            "Reduza ACTIVITY_THR ou MIN_SEG_LEN."
        )

    print(f"[4/6] Construindo features ARX  (na={NA}, nb={NB})")
    X, y_target = build_arx_features(df_dyn, na=NA, nb=NB)

    split = int(len(X) * TRAIN_RATIO)
    X_train, X_test = X[:split], X[split:]
    y_train, y_test = y_target[:split], y_target[split:]
    print(f"      Treino: {len(X_train)} amostras  |  Teste: {len(X_test)} amostras")

    print("[5/6] Ajustando ARX por OLS ...")
    coefs, r2_train = fit_arx(X_train, y_train)
    print(f"      R² treino = {r2_train:.4f}")

    print_transfer_function(coefs, na=NA, nb=NB, dt=dt)

    print("[6/6] Validando e gerando gráfico ...")
    stem = Path(ulg_path).stem
    save_path = f"artifacts/sysid_roll_{stem}.png"
    r2_test = validate(coefs, X_test, y_test, dt=dt, save_path=save_path)
    print(f"      R² validação = {r2_test:.4f}\n")


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else ULG_PATH
    main(path)
