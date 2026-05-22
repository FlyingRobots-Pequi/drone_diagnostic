# Sysid Roll ARX — Design Spec
**Date:** 2026-05-22  
**Status:** Approved  

---

## Objetivo

Identificar a função de transferência discreta do eixo de roll de um drone PX4 a partir de logs `.ulg`, usando um modelo ARX (AutoRegressive with eXogenous input), para uso posterior na sintonia de PID.

---

## Sinais

| Papel | Tópico ULG | Campo | Unidade |
|---|---|---|---|
| Entrada u(t) | `vehicle_torque_setpoint` | `xyz[0]` | Nm |
| Saída y(t) | `vehicle_angular_velocity` | `xyz[0]` | rad/s |

**Justificativa:** `actuator_controls_0` não existe no firmware PX4 recente (Pixhawk 6X). `vehicle_torque_setpoint.xyz[0]` é a saída do controlador de taxa de roll antes do alocador de motores — representa o torque comandado à planta.

---

## Arquitetura (sysid_roll.py)

```
load_and_align(ulg_path)
  └── pyulog → DataFrame por tópico
  └── timestamp µs → pd.DatetimeIndex
  └── .resample("4ms").mean().interpolate()  # 250 Hz
  └── pd.concat(axis=1, join="inner")
  └── dropna()

zero_phase_filter(df, u_col, y_col, cutoff_hz, fs)
  └── scipy.signal.butter(4, cutoff/(fs/2), btype="low")
  └── scipy.signal.filtfilt(b, a, signal)  # fase zero

select_dynamic_segments(df, y_col, threshold, min_len)
  └── |dω/dt| > threshold → máscara booleana
  └── remove segmentos < min_len amostras
  └── retorna df filtrado

build_arx_features(df, u_col, y_col, na, nb)
  └── X = [y(t-1)..y(t-na), u(t-1)..u(t-nb)]
  └── target = y(t)
  └── dropna() pós-lag

fit_arx(X_train, y_train)
  └── sklearn.LinearRegression().fit()
  └── retorna coeficientes [a1..ana, b1..bnb]

validate(model, X_test, y_test, dt)
  └── R² score
  └── Plot: roll rate real vs predito
  └── Plot: erro de predição ao longo do tempo

print_transfer_function(coefs, na, nb, dt)
  └── Exibe H(z) = B(z)/A(z) em forma polinomial
  └── Exibe polos e zeros em z

main(ulg_path)
  └── orquestra pipeline completo
  └── salva figura em artifacts/sysid_roll_<stem>.png
```

---

## Parâmetros Configuráveis

```python
ULG_PATH      = "px4_logs/pixhawk-6X/dia_13-07/log_1_2024-7-13-15-12-16.ulg"
RESAMPLE_FREQ = "4ms"     # 250 Hz
CUTOFF_HZ     = 30.0      # Butterworth LP cutoff
FILTER_ORDER  = 4         # ordem do filtro
ACTIVITY_THR  = 0.05      # rad/s² — |dω/dt| mínimo
MIN_SEG_LEN   = 50        # amostras mínimas por segmento dinâmico
NA, NB        = 2, 2      # ordem ARX (lags de y e u)
TRAIN_RATIO   = 0.7       # split temporal para validação
```

---

## Matemática — Modelo ARX e Função de Transferência

### Modelo no tempo

```
y(t) = a₁·y(t-1) + a₂·y(t-2) + b₁·u(t-1) + b₂·u(t-2) + e(t)
```

Onde `e(t)` é o resíduo (não modelado + ruído).

### Função de Transferência Discreta

Aplicando a Transformada Z:

```
Y(z) · [1 - a₁z⁻¹ - a₂z⁻²] = U(z) · [b₁z⁻¹ + b₂z⁻²]
```

Portanto:

```
H(z) = Y(z)/U(z) = (b₁z⁻¹ + b₂z⁻²) / (1 - a₁z⁻¹ - a₂z⁻²)
                 = (b₁z + b₂) / (z² - a₁z - a₂)
```

**Interpretação:**
- **Polos** de `z² - a₁z - a₂ = 0` → frequência natural e amortecimento do eixo de roll
- **Ganho DC** (z→1): `(b₁+b₂)/(1-a₁-a₂)` → rad/s por Nm em regime permanente
- Para sintonia de PID: converter H(z) → H(s) via bilinear transform (Tustin) e usar critérios de margem de fase/ganho

---

## Saídas

1. `artifacts/sysid_roll_<log_stem>.png` — 3 subplots: roll rate real vs predito, erro, espectro do resíduo
2. Impressão no terminal: coeficientes, R², H(z), polos/zeros
