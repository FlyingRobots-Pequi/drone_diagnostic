# PID Optimizer Design

**Date**: 2026-05-10
**Status**: Approved

## Context

The drone (Hermit, Pixhawk 6X) flies without GPS using optical flow + EKF2. PID gains are currently
tuned manually by trial-and-error across multiple flight sessions. The goal is to build an
offline optimization pipeline that:

1. Extracts drone dynamics from real flight logs (system identification)
2. Simulates the full PX4 cascaded PID controller in Python
3. Uses a genetic algorithm to evolve gain sets scored by setpoint-tracking RMSE
4. Exports the best candidate as a `.params` file for QGroundControl

The approach is exploration-first: Jupyter notebooks for validation at each stage, automation later.

---

## Repository Layout

```
ulg_analysis/
├── notebooks/
│   ├── 01_ulg_explorer.ipynb        # Explore all signals available in .ulg files
│   ├── 02_sysid.ipynb               # Identify drone plant model from flight data
│   ├── 03_px4_simulator.ipynb       # Reimplement + validate PX4 PID cascade
│   ├── 04_ga_optimizer.ipynb        # Genetic algorithm: evolve + score gain sets
│   └── 05_params_export.ipynb       # Map best gains → .params file
├── artifacts/
│   ├── plant_model.json             # Output of 02_sysid (drone dynamics)
│   ├── ga_checkpoint.json           # GA progress saved every 10 generations
│   └── best_gains.json              # Top-N candidates from GA
├── ulg_ingestor/                    # Existing (unchanged)
└── px4_logs/                        # Input data (gitignored)
```

---

## Notebook 1 — ULG Explorer (`01_ulg_explorer.ipynb`)

**Purpose**: Understand what data is available across all .ulg files before doing any analysis.

**Inputs**: Any `.ulg` file from `px4_logs/`
**Outputs**: None (exploratory)

**Content**:
- List all message types and fields in a log using `pyulog.ULog`
- Plot key signals: actuator outputs, IMU, attitude, position, rates
- Identify which logs have the signals needed for sysID (requires `actuator_outputs` + IMU)
- Flag the `testes_PID_position_31-08` logs as primary sysID candidates (Hermit drone, in-flight maneuvers)
- Show topic availability heatmap across all log files

**Key signals confirmed present in Hermit logs**:
- `actuator_outputs` — motor PWM commands
- `vehicle_angular_velocity` — filtered body rates (rad/s)
- `sensor_combined` — raw IMU (accel + gyro)
- `vehicle_attitude` — quaternion attitude
- `vehicle_local_position` — position + velocity in local frame
- `vehicle_rates_setpoint` — commanded angular rates
- `vehicle_attitude_setpoint` — commanded attitude
- `vehicle_local_position_setpoint` — commanded position/velocity

---

## Notebook 2 — System Identification (`02_sysid.ipynb`)

**Purpose**: Fit a plant model from closed-loop flight data.

**Inputs**: Selected `.ulg` files (prefer maneuver-rich logs from `testes_PID_position_31-08`)
**Outputs**: `artifacts/plant_model.json`

**Models to fit**:

| Model | Inputs | Method |
|---|---|---|
| Thrust coefficient `kT` | `actuator_outputs` + hover condition | Hover equilibrium: total thrust = mass × g |
| Moment of inertia `Ixx, Iyy, Izz` | Differential motor commands + angular acceleration | Least squares: τ = I × α |
| Motor time constant `τ_motor` | PWM step changes → rate transients | First-order lag fit |
| Aerodynamic drag | Velocity decay during near-glide segments | Exponential fit on `vehicle_local_position` |

**Validation**: Replay logged motor commands through the identified model, overlay simulated
angular rates vs real logged rates. Fit is acceptable when RMSE < 10% of signal range.

**`plant_model.json` schema**:
```json
{
  "mass_kg": 1.8,
  "kT": 8.5e-6,
  "tau_motor_s": 0.03,
  "inertia": {"Ixx": 0.012, "Iyy": 0.013, "Izz": 0.022},
  "drag": {"kD_xy": 0.15, "kD_z": 0.20},
  "arm_length_m": 0.17,
  "source_log": "log_12_2024-8-30-16-21-54.ulg",
  "fit_rmse": {"rates": 0.04}
}
```

---

## Notebook 3 — PX4 Simulator (`03_px4_simulator.ipynb`)

**Purpose**: Reimplement PX4's cascaded MC controller in Python and validate it against real flight data.

**Inputs**: `plant_model.json`, any `.ulg` file
**Outputs**: `pid_simulator.py` (extracted module, imported by notebook 04)

**PX4 cascade (outer → inner)**:
```
Position setpoint (x,y,z)
    │  P: MPC_XY_P, MPC_Z_P
    ▼
Velocity setpoint (vx,vy,vz)
    │  PID: MPC_XY_VEL_P/I/D_ACC, MPC_Z_VEL_P/I/D_ACC
    ▼
Attitude setpoint (roll, pitch, yaw)
    │  P: MC_ROLL_P, MC_PITCH_P, MC_YAW_P
    ▼
Rate setpoint (p, q, r)
    │  PID×K: MC_ROLLRATE_P/I/D×K, MC_PITCHRATE_P/I/D×K, MC_YAWRATE_P/I/D×K
    ▼
Actuator torques → motor mixing → PWM
```

**PX4 source references**:
- `src/modules/mc_rate_control/MulticopterRateControl.cpp`
- `src/modules/mc_att_control/AttitudeControl.cpp`
- `src/modules/mc_pos_control/PositionControl/PositionControl.cpp`

**Validation**: Replay logged position setpoints as inputs. Compare simulated state trajectory
vs real logged state. Accept when trajectory RMSE < 15% of maneuver amplitude.

**`pid_simulator.py` interface**:
```python
sim = PX4Simulator(plant_model, gains)
result = sim.run(setpoints_df, dt=0.004)   # returns DataFrame of simulated states
rmse = compute_rmse(result, actual_df)
```

---

## Notebook 4 — Genetic Algorithm (`04_ga_optimizer.ipynb`)

**Purpose**: Evolve PID gain sets and score them by setpoint-tracking RMSE using the simulator.

**Inputs**: `pid_simulator.py`, `plant_model.json`, reference setpoints from a real flight log
**Outputs**: `artifacts/ga_checkpoint.json` (every 10 generations), `artifacts/best_gains.json`

### Search Space (28 parameters)

| Loop | Parameters | Bounds |
|---|---|---|
| Rate roll/pitch | P, I, D, K | P:[0.01–0.5] I:[0.0–0.5] D:[0.0–0.012] K:[0.1–3.0] |
| Rate yaw | P, I, D, K | P:[0.01–0.5] I:[0.0–0.5] D:[0.0–0.010] K:[0.1–3.0] |
| Attitude roll/pitch | P | [1.0–12.0] |
| Attitude yaw | P | [0.5–5.0] |
| Velocity XY | P, I, D (ACC) | P:[0.1–5.0] I:[0.0–5.0] D:[0.0–2.0] |
| Velocity Z | P, I, D (ACC) | P:[0.1–8.0] I:[0.0–5.0] D:[0.0–2.0] |
| Position XY | P | [0.1–2.0] |
| Position Z | P | [0.1–2.0] |

### GA Configuration

- Population: 100 individuals
- Generations: 200 (resumable from checkpoint)
- Selection: tournament (size=5)
- Crossover: uniform (probability 0.7)
- Mutation: Gaussian perturbation (σ = 5% of bound range, probability per gene = 0.2)
- Fitness: weighted RMSE across all loops (rate > attitude > velocity > position, weights TBD in notebook)

### Visualizations
- Fitness convergence plot per generation (best + mean + std)
- Best individual's simulated step response vs setpoint (all axes)
- Parameter heat map: which gains converged tightly vs stayed spread

---

## Notebook 5 — Params Export (`05_params_export.ipynb`)

**Purpose**: Convert best GA candidate(s) to a QGroundControl-compatible `.params` file.

**Inputs**: `best_gains.json`, existing params file as baseline
**Outputs**: `optimized_<drone>_<date>.params`

**Steps**:
1. Load top-N candidates from `best_gains.json`
2. Show comparison table: current vs optimized, % change per parameter
3. Flag any gain that changed >50% from reference value as a warning
4. Plot simulated step response: current vs optimized side by side
5. User selects candidate to export
6. Write `.params` — only the 28 tuned parameters; all others copied unchanged from baseline

**Output format** (QGroundControl compatible):
```
# PID Optimizer — Hermit — 2026-05-10 — fitness RMSE: 0.031
1	1	MC_ROLLRATE_P	0.230000	9
1	1	MC_ROLLRATE_I	0.190000	9
...
```

---

## Dependencies

```
pyulog>=0.9.0
numpy>=1.21.0
scipy>=1.9.0
pandas>=1.5.0
matplotlib>=3.6.0
jupyter
```

No InfluxDB dependency for this phase — all intermediate artifacts are JSON/CSV files.

---

## Verification

1. **Notebook 1**: Run on `log_12_2024-8-30-16-21-54.ulg` — all key signals should appear in the topic list
2. **Notebook 2**: Simulated angular rates overlay on real rates — visual match
3. **Notebook 3**: Simulated trajectory replays a real flight within 15% RMSE
4. **Notebook 4**: GA fitness decreases monotonically over generations; final RMSE < initial
5. **Notebook 5**: Output `.params` loads in QGroundControl without errors
