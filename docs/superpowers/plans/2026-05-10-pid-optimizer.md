# PID Optimizer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a 5-notebook pipeline that identifies drone dynamics from PX4 flight logs, simulates the full cascaded PID controller, and runs a genetic algorithm to produce an optimized `.params` file.

**Architecture:** Python helper modules (`pid_optimizer/`) contain all testable logic; Jupyter notebooks import them for exploration and validation. Intermediate outputs are JSON files in `artifacts/`. No InfluxDB dependency in this phase.

**Tech Stack:** Python 3.11, pyulog, numpy, scipy, pandas, matplotlib, jupyter, pytest

---

## File Map

```
ulg_analysis/
├── pid_optimizer/
│   ├── __init__.py
│   ├── plant_model.py     # PlantModel dataclass + JSON I/O
│   ├── gains.py           # Gains dataclass (23 PID params) + PX4 param mapping
│   ├── pid_simulator.py   # PX4 cascade + rigid body dynamics
│   ├── ga.py              # Genetic algorithm
│   └── params_io.py       # .params file parser and writer
├── tests/
│   ├── test_plant_model.py
│   ├── test_gains.py
│   ├── test_pid_simulator.py
│   ├── test_ga.py
│   └── test_params_io.py
├── notebooks/
│   ├── 01_ulg_explorer.ipynb
│   ├── 02_sysid.ipynb
│   ├── 03_px4_simulator.ipynb
│   ├── 04_ga_optimizer.ipynb
│   └── 05_params_export.ipynb
├── artifacts/             # gitignored — generated outputs
│   ├── plant_model.json
│   ├── ga_checkpoint.json
│   └── best_gains.json
└── requirements_notebooks.txt
```

---

## Task 1: Environment Setup

**Files:**
- Create: `ulg_analysis/requirements_notebooks.txt`
- Create: `ulg_analysis/pid_optimizer/__init__.py`
- Create: `ulg_analysis/.gitignore` entry for `artifacts/`

- [ ] **Step 1: Create requirements file**

```
# ulg_analysis/requirements_notebooks.txt
pyulog>=0.9.0
numpy>=1.21.0
scipy>=1.9.0
pandas>=1.5.0
matplotlib>=3.6.0
jupyter
pytest
```

- [ ] **Step 2: Create package init**

```python
# ulg_analysis/pid_optimizer/__init__.py
from .plant_model import PlantModel, Inertia, Drag
from .gains import Gains, GAIN_BOUNDS, HERMIT_REFERENCE_GAINS
from .pid_simulator import PX4Simulator, compute_rmse
from .ga import GeneticAlgorithm
from .params_io import load_params, save_params
```

- [ ] **Step 3: Add artifacts/ to gitignore**

Append to `ulg_analysis/.gitignore` (create if missing):
```
artifacts/
*.params
```

- [ ] **Step 4: Verify environment works**

```bash
cd ulg_analysis
uv run --with pyulog,numpy,scipy,pandas,matplotlib,pytest python -c "import pyulog, numpy, scipy, pandas, matplotlib; print('OK')"
```
Expected: `OK`

- [ ] **Step 5: Commit**

```bash
git add ulg_analysis/requirements_notebooks.txt ulg_analysis/pid_optimizer/__init__.py
git commit -m "feat: scaffold pid_optimizer package"
```

---

## Task 2: PlantModel Data Structure

**Files:**
- Create: `ulg_analysis/pid_optimizer/plant_model.py`
- Create: `ulg_analysis/tests/test_plant_model.py`

- [ ] **Step 1: Write failing test**

```python
# ulg_analysis/tests/test_plant_model.py
import json, tempfile, os
from pid_optimizer.plant_model import PlantModel, Inertia, Drag

def test_roundtrip_json():
    model = PlantModel(
        mass_kg=1.8,
        kT=8.5e-6,
        tau_motor_s=0.03,
        inertia=Inertia(Ixx=0.012, Iyy=0.013, Izz=0.022),
        drag=Drag(kD_xy=0.15, kD_z=0.20),
        arm_length_m=0.17,
        source_log="test.ulg",
        fit_rmse={"rates": 0.04},
    )
    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
        path = f.name
    try:
        model.save(path)
        loaded = PlantModel.load(path)
        assert loaded.mass_kg == model.mass_kg
        assert loaded.inertia.Ixx == model.inertia.Ixx
        assert loaded.drag.kD_xy == model.drag.kD_xy
        assert loaded.fit_rmse == model.fit_rmse
    finally:
        os.unlink(path)
```

- [ ] **Step 2: Run to verify it fails**

```bash
cd ulg_analysis
uv run --with pyulog,numpy,scipy,pandas,pytest pytest tests/test_plant_model.py -v
```
Expected: `FAILED` — `ModuleNotFoundError: No module named 'pid_optimizer.plant_model'`

- [ ] **Step 3: Implement PlantModel**

```python
# ulg_analysis/pid_optimizer/plant_model.py
from dataclasses import dataclass, asdict
import json


@dataclass
class Inertia:
    Ixx: float
    Iyy: float
    Izz: float


@dataclass
class Drag:
    kD_xy: float
    kD_z: float


@dataclass
class PlantModel:
    mass_kg: float
    kT: float           # N per (normalized_cmd^2) — fitted from hover
    tau_motor_s: float  # first-order motor lag time constant
    inertia: Inertia
    drag: Drag
    arm_length_m: float
    source_log: str = ""
    fit_rmse: dict = None

    def save(self, path: str) -> None:
        with open(path, "w") as f:
            json.dump(asdict(self), f, indent=2)

    @classmethod
    def load(cls, path: str) -> "PlantModel":
        with open(path) as f:
            d = json.load(f)
        d["inertia"] = Inertia(**d["inertia"])
        d["drag"] = Drag(**d["drag"])
        return cls(**d)
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd ulg_analysis
uv run --with pyulog,numpy,scipy,pandas,pytest pytest tests/test_plant_model.py -v
```
Expected: `PASSED`

- [ ] **Step 5: Commit**

```bash
git add ulg_analysis/pid_optimizer/plant_model.py ulg_analysis/tests/test_plant_model.py
git commit -m "feat: add PlantModel dataclass with JSON I/O"
```

---

## Task 3: Gains Data Structure

**Files:**
- Create: `ulg_analysis/pid_optimizer/gains.py`
- Create: `ulg_analysis/tests/test_gains.py`

- [ ] **Step 1: Write failing tests**

```python
# ulg_analysis/tests/test_gains.py
import numpy as np
from pid_optimizer.gains import Gains, GAIN_BOUNDS, HERMIT_REFERENCE_GAINS

def test_to_array_from_array_roundtrip():
    g = HERMIT_REFERENCE_GAINS
    arr = g.to_array()
    g2 = Gains.from_array(arr)
    assert np.allclose(arr, g2.to_array())

def test_array_length():
    assert len(HERMIT_REFERENCE_GAINS.to_array()) == 23

def test_bounds_length():
    assert len(GAIN_BOUNDS) == 23

def test_to_px4_params_keys():
    params = HERMIT_REFERENCE_GAINS.to_px4_params()
    expected_keys = {
        "MC_ROLLRATE_P", "MC_ROLLRATE_I", "MC_ROLLRATE_D", "MC_ROLLRATE_K",
        "MC_PITCHRATE_P", "MC_PITCHRATE_I", "MC_PITCHRATE_D", "MC_PITCHRATE_K",
        "MC_YAWRATE_P", "MC_YAWRATE_I", "MC_YAWRATE_D", "MC_YAWRATE_K",
        "MC_ROLL_P", "MC_PITCH_P", "MC_YAW_P",
        "MPC_XY_VEL_P_ACC", "MPC_XY_VEL_I_ACC", "MPC_XY_VEL_D_ACC",
        "MPC_Z_VEL_P_ACC", "MPC_Z_VEL_I_ACC", "MPC_Z_VEL_D_ACC",
        "MPC_XY_P", "MPC_Z_P",
    }
    assert set(params.keys()) == expected_keys
```

- [ ] **Step 2: Run to verify it fails**

```bash
cd ulg_analysis
uv run --with pyulog,numpy,scipy,pandas,pytest pytest tests/test_gains.py -v
```
Expected: `FAILED` — `ModuleNotFoundError`

- [ ] **Step 3: Implement Gains**

```python
# ulg_analysis/pid_optimizer/gains.py
from dataclasses import dataclass, fields
import numpy as np


@dataclass
class Gains:
    # Rate controller (PID + output multiplier K)
    rollrate_P: float = 0.215
    rollrate_I: float = 0.175
    rollrate_D: float = 0.0022
    rollrate_K: float = 0.45
    pitchrate_P: float = 0.215
    pitchrate_I: float = 0.175
    pitchrate_D: float = 0.0022
    pitchrate_K: float = 0.45
    yawrate_P: float = 0.154
    yawrate_I: float = 0.160
    yawrate_D: float = 0.0035
    yawrate_K: float = 1.75
    # Attitude controller (P-only per axis)
    roll_P: float = 7.0
    pitch_P: float = 7.0
    yaw_P: float = 5.0
    # Velocity controller (output in m/s²)
    xy_vel_P: float = 2.05
    xy_vel_I: float = 3.0
    xy_vel_D: float = 0.2
    z_vel_P: float = 4.0
    z_vel_I: float = 2.0
    z_vel_D: float = 0.0
    # Position controller (P-only)
    xy_pos_P: float = 1.1
    z_pos_P: float = 1.0

    def to_array(self) -> np.ndarray:
        return np.array([getattr(self, f.name) for f in fields(self)], dtype=float)

    @classmethod
    def from_array(cls, arr: np.ndarray) -> "Gains":
        return cls(**{f.name: float(v) for f, v in zip(fields(cls), arr)})

    def to_px4_params(self) -> dict:
        return {
            "MC_ROLLRATE_P": self.rollrate_P,
            "MC_ROLLRATE_I": self.rollrate_I,
            "MC_ROLLRATE_D": self.rollrate_D,
            "MC_ROLLRATE_K": self.rollrate_K,
            "MC_PITCHRATE_P": self.pitchrate_P,
            "MC_PITCHRATE_I": self.pitchrate_I,
            "MC_PITCHRATE_D": self.pitchrate_D,
            "MC_PITCHRATE_K": self.pitchrate_K,
            "MC_YAWRATE_P": self.yawrate_P,
            "MC_YAWRATE_I": self.yawrate_I,
            "MC_YAWRATE_D": self.yawrate_D,
            "MC_YAWRATE_K": self.yawrate_K,
            "MC_ROLL_P": self.roll_P,
            "MC_PITCH_P": self.pitch_P,
            "MC_YAW_P": self.yaw_P,
            "MPC_XY_VEL_P_ACC": self.xy_vel_P,
            "MPC_XY_VEL_I_ACC": self.xy_vel_I,
            "MPC_XY_VEL_D_ACC": self.xy_vel_D,
            "MPC_Z_VEL_P_ACC": self.z_vel_P,
            "MPC_Z_VEL_I_ACC": self.z_vel_I,
            "MPC_Z_VEL_D_ACC": self.z_vel_D,
            "MPC_XY_P": self.xy_pos_P,
            "MPC_Z_P": self.z_pos_P,
        }


# (low, high) bounds per gain, same order as Gains fields
GAIN_BOUNDS = [
    (0.01, 0.5),   # rollrate_P
    (0.0,  0.5),   # rollrate_I
    (0.0,  0.012), # rollrate_D
    (0.1,  3.0),   # rollrate_K
    (0.01, 0.5),   # pitchrate_P
    (0.0,  0.5),   # pitchrate_I
    (0.0,  0.012), # pitchrate_D
    (0.1,  3.0),   # pitchrate_K
    (0.01, 0.5),   # yawrate_P
    (0.0,  0.5),   # yawrate_I
    (0.0,  0.010), # yawrate_D
    (0.1,  3.0),   # yawrate_K
    (1.0,  12.0),  # roll_P
    (1.0,  12.0),  # pitch_P
    (0.5,  5.0),   # yaw_P
    (0.1,  5.0),   # xy_vel_P
    (0.0,  5.0),   # xy_vel_I
    (0.0,  2.0),   # xy_vel_D
    (0.1,  8.0),   # z_vel_P
    (0.0,  5.0),   # z_vel_I
    (0.0,  2.0),   # z_vel_D
    (0.1,  2.0),   # xy_pos_P
    (0.1,  2.0),   # z_pos_P
]

# Hermit reference gains (from log_12_2024-8-30-16-21-54.ulg)
HERMIT_REFERENCE_GAINS = Gains()
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd ulg_analysis
uv run --with pyulog,numpy,scipy,pandas,pytest pytest tests/test_gains.py -v
```
Expected: all 4 `PASSED`

- [ ] **Step 5: Commit**

```bash
git add ulg_analysis/pid_optimizer/gains.py ulg_analysis/tests/test_gains.py
git commit -m "feat: add Gains dataclass with PX4 param mapping and search bounds"
```

---

## Task 4: PX4 Simulator Module

**Files:**
- Create: `ulg_analysis/pid_optimizer/pid_simulator.py`
- Create: `ulg_analysis/tests/test_pid_simulator.py`

- [ ] **Step 1: Write failing tests**

```python
# ulg_analysis/tests/test_pid_simulator.py
import numpy as np
import pandas as pd
from pid_optimizer.plant_model import PlantModel, Inertia, Drag
from pid_optimizer.gains import Gains
from pid_optimizer.pid_simulator import PX4Simulator, compute_rmse

PLANT = PlantModel(
    mass_kg=1.8, kT=8.5e-6, tau_motor_s=0.03,
    inertia=Inertia(Ixx=0.012, Iyy=0.013, Izz=0.022),
    drag=Drag(kD_xy=0.15, kD_z=0.20),
    arm_length_m=0.17,
)
GAINS = Gains()


def test_rate_controller_converges_to_setpoint():
    """Pure P rate controller steps from 0 to 0.5 rad/s; error should shrink."""
    gains = Gains(rollrate_P=0.3, rollrate_I=0.0, rollrate_D=0.0, rollrate_K=1.0,
                  pitchrate_P=0.3, pitchrate_I=0.0, pitchrate_D=0.0, pitchrate_K=1.0,
                  yawrate_P=0.3, yawrate_I=0.0, yawrate_D=0.0, yawrate_K=1.0,
                  roll_P=6.0, pitch_P=6.0, yaw_P=4.0,
                  xy_vel_P=2.0, xy_vel_I=0.0, xy_vel_D=0.0,
                  z_vel_P=4.0, z_vel_I=0.0, z_vel_D=0.0,
                  xy_pos_P=1.0, z_pos_P=1.0)
    sim = PX4Simulator(PLANT, gains)
    n_steps = 500
    dt = 0.004
    setpoints = pd.DataFrame({
        "x": [0.0] * n_steps, "y": [0.0] * n_steps, "z": [-1.0] * n_steps,
        "vx": [0.0] * n_steps, "vy": [0.0] * n_steps, "vz": [0.0] * n_steps,
        "roll": [0.1] * n_steps, "pitch": [0.0] * n_steps, "yaw": [0.0] * n_steps,
    })
    result = sim.run(setpoints, dt=dt)
    # Roll angle should move toward 0.1 rad setpoint
    initial_err = abs(0.1 - result["roll"].iloc[0])
    final_err = abs(0.1 - result["roll"].iloc[-1])
    assert final_err < initial_err, f"Roll did not converge: initial={initial_err:.4f} final={final_err:.4f}"


def test_hover_stays_near_origin():
    """Drone commanded to hover at [0,0,-1] starting from rest should stay close."""
    sim = PX4Simulator(PLANT, GAINS)
    n_steps = 500
    setpoints = pd.DataFrame({
        "x": [0.0] * n_steps, "y": [0.0] * n_steps, "z": [-1.0] * n_steps,
        "vx": [0.0] * n_steps, "vy": [0.0] * n_steps, "vz": [0.0] * n_steps,
        "roll": [0.0] * n_steps, "pitch": [0.0] * n_steps, "yaw": [0.0] * n_steps,
    })
    result = sim.run(setpoints, dt=0.004)
    # After 2 seconds (500 steps × 0.004s), position should be within 1m of setpoint
    final_z_err = abs(-1.0 - result["z"].iloc[-1])
    assert final_z_err < 1.0, f"Z position drifted too far: {final_z_err:.2f} m"


def test_compute_rmse():
    a = pd.DataFrame({"x": [1.0, 2.0, 3.0], "y": [0.0, 0.0, 0.0]})
    b = pd.DataFrame({"x": [1.0, 2.0, 3.0], "y": [1.0, 1.0, 1.0]})
    rmse = compute_rmse(a, b, cols=["x", "y"])
    assert abs(rmse["x"] - 0.0) < 1e-9
    assert abs(rmse["y"] - 1.0) < 1e-9
```

- [ ] **Step 2: Run to verify they fail**

```bash
cd ulg_analysis
uv run --with pyulog,numpy,scipy,pandas,pytest pytest tests/test_pid_simulator.py -v
```
Expected: `FAILED` — `ModuleNotFoundError`

- [ ] **Step 3: Implement the simulator**

```python
# ulg_analysis/pid_optimizer/pid_simulator.py
import numpy as np
import pandas as pd
from dataclasses import dataclass
from .plant_model import PlantModel
from .gains import Gains


def _euler_to_R(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX rotation matrix: world → body."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,    cp*sr,             cp*cr],
    ])


class _RateController:
    def __init__(self, P, I, D, K, dt, integral_limit=0.3):
        self.P, self.I, self.D, self.K = P, I, D, K
        self.dt = dt
        self._integral = np.zeros(3)
        self._prev_rate = np.zeros(3)
        self._integral_limit = integral_limit

    def reset(self):
        self._integral[:] = 0.0
        self._prev_rate[:] = 0.0

    def update(self, setpoint: np.ndarray, actual: np.ndarray) -> np.ndarray:
        error = setpoint - actual
        self._integral += error * self.dt
        self._integral = np.clip(self._integral, -self._integral_limit, self._integral_limit)
        # D on measurement to avoid derivative kick on setpoint change
        d_term = -(actual - self._prev_rate) / self.dt
        self._prev_rate = actual.copy()
        P_vec = np.array([self.P[0], self.P[1], self.P[2]])
        I_vec = np.array([self.I[0], self.I[1], self.I[2]])
        D_vec = np.array([self.D[0], self.D[1], self.D[2]])
        K_vec = np.array([self.K[0], self.K[1], self.K[2]])
        return K_vec * (P_vec * error + I_vec * self._integral + D_vec * d_term)


class _AttitudeController:
    def __init__(self, roll_P, pitch_P, yaw_P, rate_limit=np.deg2rad(220)):
        self.P = np.array([roll_P, pitch_P, yaw_P])
        self.rate_limit = rate_limit

    def update(self, setpoint_euler: np.ndarray, actual_euler: np.ndarray) -> np.ndarray:
        error = setpoint_euler - actual_euler
        error[2] = (error[2] + np.pi) % (2 * np.pi) - np.pi  # wrap yaw
        rate_sp = self.P * error
        return np.clip(rate_sp, -self.rate_limit, self.rate_limit)


class _VelocityController:
    def __init__(self, xy_P, xy_I, xy_D, z_P, z_I, z_D, dt, integral_limit=5.0):
        self.P = np.array([xy_P, xy_P, z_P])
        self.I = np.array([xy_I, xy_I, z_I])
        self.D = np.array([xy_D, xy_D, z_D])
        self.dt = dt
        self._integral = np.zeros(3)
        self._prev_error = np.zeros(3)
        self._integral_limit = integral_limit

    def reset(self):
        self._integral[:] = 0.0
        self._prev_error[:] = 0.0

    def update(self, vel_sp: np.ndarray, vel_actual: np.ndarray) -> np.ndarray:
        error = vel_sp - vel_actual
        self._integral += error * self.dt
        self._integral = np.clip(self._integral, -self._integral_limit, self._integral_limit)
        d_term = (error - self._prev_error) / self.dt
        self._prev_error = error.copy()
        acc_sp = self.P * error + self.I * self._integral + self.D * d_term
        return acc_sp


class _PositionController:
    def __init__(self, xy_P, z_P, xy_vel_max=12.0, z_vel_max_up=3.0, z_vel_max_dn=1.5):
        self.P = np.array([xy_P, xy_P, z_P])
        self._vel_max = np.array([xy_vel_max, xy_vel_max, max(z_vel_max_up, z_vel_max_dn)])

    def update(self, pos_sp: np.ndarray, pos_actual: np.ndarray) -> np.ndarray:
        vel_sp = self.P * (pos_sp - pos_actual)
        return np.clip(vel_sp, -self._vel_max, self._vel_max)


class PX4Simulator:
    """Simulate PX4 cascaded MC controller + rigid body plant."""

    def __init__(self, plant: PlantModel, gains: Gains):
        self.plant = plant
        self.gains = gains

    def run(self, setpoints: pd.DataFrame, dt: float = 0.004) -> pd.DataFrame:
        """
        setpoints: DataFrame with columns [x, y, z, vx, vy, vz, roll, pitch, yaw]
        Returns DataFrame with simulated state at each timestep:
            [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]
        """
        g = self.gains
        p = self.plant

        pos_ctrl = _PositionController(g.xy_pos_P, g.z_pos_P)
        vel_ctrl = _VelocityController(g.xy_vel_P, g.xy_vel_I, g.xy_vel_D,
                                        g.z_vel_P, g.z_vel_I, g.z_vel_D, dt)
        att_ctrl = _AttitudeController(g.roll_P, g.pitch_P, g.yaw_P)
        rate_ctrl = _RateController(
            P=[g.rollrate_P, g.pitchrate_P, g.yawrate_P],
            I=[g.rollrate_I, g.pitchrate_I, g.yawrate_I],
            D=[g.rollrate_D, g.pitchrate_D, g.yawrate_D],
            K=[g.rollrate_K, g.pitchrate_K, g.yawrate_K],
            dt=dt,
        )

        # State: [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]
        state = np.zeros(12)
        G = 9.81
        I_vec = np.array([p.inertia.Ixx, p.inertia.Iyy, p.inertia.Izz])
        max_thrust = 2.0 * p.mass_kg * G  # max thrust ~ 2× weight

        records = []
        for i in range(len(setpoints)):
            sp = setpoints.iloc[i]
            pos = state[0:3]
            vel = state[3:6]
            euler = state[6:9]
            omega = state[9:12]

            pos_sp = np.array([sp["x"], sp["y"], sp["z"]])
            vel_sp = pos_ctrl.update(pos_sp, pos)
            vel_sp += np.array([sp["vx"], sp["vy"], sp["vz"]])  # feed-forward

            acc_sp = vel_ctrl.update(vel_sp, vel)

            # Acceleration setpoint → attitude setpoint (simplified: tilt for XY, thrust for Z)
            thrust_sp = np.clip((acc_sp[2] + G) * p.mass_kg, 0.0, max_thrust)
            roll_sp_from_acc = np.clip(np.arctan2(-acc_sp[1], G), -np.deg2rad(45), np.deg2rad(45))
            pitch_sp_from_acc = np.clip(np.arctan2(acc_sp[0], G), -np.deg2rad(45), np.deg2rad(45))
            att_sp = np.array([roll_sp_from_acc, pitch_sp_from_acc, sp["yaw"]])

            rate_sp = att_ctrl.update(att_sp, euler)
            torque_cmd = rate_ctrl.update(rate_sp, omega)

            # Rigid body dynamics
            R = _euler_to_R(*euler)
            # Linear: thrust in world frame
            thrust_body = np.array([0.0, 0.0, -thrust_sp])
            a_world = R @ thrust_body / p.mass_kg + np.array([0.0, 0.0, G])
            drag_coeff = np.array([p.drag.kD_xy, p.drag.kD_xy, p.drag.kD_z])
            a_drag = -drag_coeff * vel / p.mass_kg
            a_total = a_world + a_drag

            # Rotational: τ = I×α + ω×(I×ω)
            scale = np.array([p.inertia.Ixx * 50, p.inertia.Iyy * 50, p.inertia.Izz * 20])
            tau = torque_cmd * scale
            alpha = (tau - np.cross(omega, I_vec * omega)) / I_vec

            # First-order motor lag on effective torque (simplified)
            alpha *= (dt / (p.tau_motor_s + dt))

            # Integrate
            state[0:3] = pos + vel * dt
            state[3:6] = vel + a_total * dt
            state[6:9] = euler + omega * dt
            state[9:12] = omega + alpha * dt

            records.append(state.copy())

        cols = ["x", "y", "z", "vx", "vy", "vz", "roll", "pitch", "yaw", "p", "q", "r"]
        return pd.DataFrame(records, columns=cols)


def compute_rmse(simulated: pd.DataFrame, reference: pd.DataFrame,
                  cols: list = None) -> dict:
    """RMSE per column between two DataFrames of the same length."""
    if cols is None:
        cols = simulated.columns.tolist()
    n = min(len(simulated), len(reference))
    return {
        col: float(np.sqrt(np.mean((simulated[col].values[:n] - reference[col].values[:n]) ** 2)))
        for col in cols
    }
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd ulg_analysis
uv run --with pyulog,numpy,scipy,pandas,pytest pytest tests/test_pid_simulator.py -v
```
Expected: all 3 `PASSED`

- [ ] **Step 5: Commit**

```bash
git add ulg_analysis/pid_optimizer/pid_simulator.py ulg_analysis/tests/test_pid_simulator.py
git commit -m "feat: implement PX4 cascaded PID simulator with rigid body dynamics"
```

---

## Task 5: Genetic Algorithm Module

**Files:**
- Create: `ulg_analysis/pid_optimizer/ga.py`
- Create: `ulg_analysis/tests/test_ga.py`

- [ ] **Step 1: Write failing tests**

```python
# ulg_analysis/tests/test_ga.py
import numpy as np
from pid_optimizer.ga import GeneticAlgorithm
from pid_optimizer.gains import GAIN_BOUNDS

# Trivial fitness: minimize sum of all gains (lower = better)
def mock_fitness(gains_array: np.ndarray) -> float:
    return float(np.sum(gains_array))

def test_tournament_selects_better():
    ga = GeneticAlgorithm(GAIN_BOUNDS, fitness_fn=mock_fitness, pop_size=10, seed=42)
    ga.initialize()
    pop = ga.population
    scores = np.array([mock_fitness(ind) for ind in pop])
    winner_idx = ga._tournament(scores, tournament_size=3)
    winner_score = scores[winner_idx]
    # Winner should be at or below median score
    assert winner_score <= np.median(scores)

def test_crossover_genes_from_parents():
    bounds = [(0.0, 1.0)] * 4
    ga = GeneticAlgorithm(bounds, fitness_fn=lambda x: 0.0, pop_size=4, seed=0)
    p1 = np.array([0.1, 0.2, 0.3, 0.4])
    p2 = np.array([0.9, 0.8, 0.7, 0.6])
    child = ga._crossover(p1, p2, prob=1.0)
    for i, v in enumerate(child):
        assert v == p1[i] or v == p2[i], f"Gene {i}={v} not from either parent"

def test_mutation_stays_in_bounds():
    bounds = [(0.0, 1.0)] * 6
    ga = GeneticAlgorithm(bounds, fitness_fn=lambda x: 0.0, pop_size=4, seed=1)
    ind = np.array([0.5] * 6)
    for _ in range(100):
        mutated = ga._mutate(ind, prob=1.0)
        lo = np.array([b[0] for b in bounds])
        hi = np.array([b[1] for b in bounds])
        assert np.all(mutated >= lo) and np.all(mutated <= hi)

def test_ga_improves_fitness():
    ga = GeneticAlgorithm(GAIN_BOUNDS, fitness_fn=mock_fitness,
                           pop_size=30, seed=7)
    ga.initialize()
    initial_best = ga.best_fitness
    for _ in range(10):
        ga.step()
    assert ga.best_fitness <= initial_best, "Fitness should decrease (minimize sum)"
```

- [ ] **Step 2: Run to verify they fail**

```bash
cd ulg_analysis
uv run --with pyulog,numpy,scipy,pandas,pytest pytest tests/test_ga.py -v
```
Expected: `FAILED` — `ModuleNotFoundError`

- [ ] **Step 3: Implement GeneticAlgorithm**

```python
# ulg_analysis/pid_optimizer/ga.py
import json
import numpy as np
from typing import Callable, List, Optional, Tuple


class GeneticAlgorithm:
    """Minimizing genetic algorithm for PID gain optimization."""

    def __init__(
        self,
        bounds: List[Tuple[float, float]],
        fitness_fn: Callable[[np.ndarray], float],
        pop_size: int = 100,
        tournament_size: int = 5,
        crossover_prob: float = 0.7,
        mutation_prob: float = 0.2,
        mutation_sigma_frac: float = 0.05,
        seed: Optional[int] = None,
    ):
        self.bounds = bounds
        self.fitness_fn = fitness_fn
        self.pop_size = pop_size
        self.tournament_size = tournament_size
        self.crossover_prob = crossover_prob
        self.mutation_prob = mutation_prob
        self.mutation_sigma_frac = mutation_sigma_frac
        self.rng = np.random.default_rng(seed)
        self.n_genes = len(bounds)

        self.population: np.ndarray = None
        self.scores: np.ndarray = None
        self.generation: int = 0
        self.history: List[dict] = []  # per-generation stats

    @property
    def best_fitness(self) -> float:
        return float(self.scores.min()) if self.scores is not None else float("inf")

    @property
    def best_individual(self) -> np.ndarray:
        return self.population[self.scores.argmin()].copy()

    def initialize(self) -> None:
        lo = np.array([b[0] for b in self.bounds])
        hi = np.array([b[1] for b in self.bounds])
        self.population = self.rng.uniform(lo, hi, size=(self.pop_size, self.n_genes))
        self.scores = np.array([self.fitness_fn(ind) for ind in self.population])
        self.generation = 0

    def _tournament(self, scores: np.ndarray, tournament_size: int) -> int:
        idx = self.rng.choice(len(scores), size=tournament_size, replace=False)
        return int(idx[scores[idx].argmin()])

    def _crossover(self, p1: np.ndarray, p2: np.ndarray, prob: float) -> np.ndarray:
        mask = self.rng.random(self.n_genes) < prob
        child = np.where(mask, p1, p2)
        return child.copy()

    def _mutate(self, ind: np.ndarray, prob: float) -> np.ndarray:
        lo = np.array([b[0] for b in self.bounds])
        hi = np.array([b[1] for b in self.bounds])
        sigma = self.mutation_sigma_frac * (hi - lo)
        mask = self.rng.random(self.n_genes) < prob
        noise = self.rng.normal(0.0, sigma)
        mutated = ind + mask * noise
        return np.clip(mutated, lo, hi)

    def step(self) -> None:
        """Advance one generation."""
        new_pop = np.empty_like(self.population)
        for i in range(self.pop_size):
            p1_idx = self._tournament(self.scores, self.tournament_size)
            p2_idx = self._tournament(self.scores, self.tournament_size)
            child = self._crossover(self.population[p1_idx], self.population[p2_idx],
                                     self.crossover_prob)
            child = self._mutate(child, self.mutation_prob)
            new_pop[i] = child
        new_scores = np.array([self.fitness_fn(ind) for ind in new_pop])

        # Elitism: keep best from previous generation
        best_prev_idx = int(self.scores.argmin())
        worst_new_idx = int(new_scores.argmax())
        new_pop[worst_new_idx] = self.population[best_prev_idx]
        new_scores[worst_new_idx] = self.scores[best_prev_idx]

        self.population = new_pop
        self.scores = new_scores
        self.generation += 1
        self.history.append({
            "generation": self.generation,
            "best": float(new_scores.min()),
            "mean": float(new_scores.mean()),
            "std": float(new_scores.std()),
        })

    def save_checkpoint(self, path: str) -> None:
        payload = {
            "generation": self.generation,
            "history": self.history,
            "best_individual": self.best_individual.tolist(),
            "best_fitness": self.best_fitness,
            "population": self.population.tolist(),
            "scores": self.scores.tolist(),
        }
        with open(path, "w") as f:
            json.dump(payload, f, indent=2)

    def load_checkpoint(self, path: str) -> None:
        with open(path) as f:
            payload = json.load(f)
        self.generation = payload["generation"]
        self.history = payload["history"]
        self.population = np.array(payload["population"])
        self.scores = np.array(payload["scores"])
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd ulg_analysis
uv run --with pyulog,numpy,scipy,pandas,pytest pytest tests/test_ga.py -v
```
Expected: all 4 `PASSED`

- [ ] **Step 5: Commit**

```bash
git add ulg_analysis/pid_optimizer/ga.py ulg_analysis/tests/test_ga.py
git commit -m "feat: implement genetic algorithm with elitism, checkpoint save/load"
```

---

## Task 6: Params I/O Module

**Files:**
- Create: `ulg_analysis/pid_optimizer/params_io.py`
- Create: `ulg_analysis/tests/test_params_io.py`

- [ ] **Step 1: Write failing tests**

```python
# ulg_analysis/tests/test_params_io.py
import tempfile, os
from pid_optimizer.params_io import load_params, save_params
from pid_optimizer.gains import Gains, HERMIT_REFERENCE_GAINS

REAL_PARAMS_PATH = "px4_logs/pixhawk-6X/dia_20-06/parametersV6x6_20.params"

def test_load_params_reads_known_value():
    params = load_params(REAL_PARAMS_PATH)
    assert "MC_ROLLRATE_P" in params
    assert abs(params["MC_ROLLRATE_P"] - 0.15) < 0.01

def test_save_params_roundtrip():
    params = load_params(REAL_PARAMS_PATH)
    with tempfile.NamedTemporaryFile(suffix=".params", delete=False, mode="w") as f:
        path = f.name
    try:
        save_params(path, params, comment="test")
        loaded = load_params(path)
        assert abs(loaded["MC_ROLLRATE_P"] - params["MC_ROLLRATE_P"]) < 1e-6
    finally:
        os.unlink(path)

def test_save_params_only_updates_tuned_keys():
    baseline = load_params(REAL_PARAMS_PATH)
    optimized_gains = Gains(rollrate_P=0.30)  # only change rollrate_P
    tuned = optimized_gains.to_px4_params()
    merged = {**baseline, **tuned}

    with tempfile.NamedTemporaryFile(suffix=".params", delete=False, mode="w") as f:
        path = f.name
    try:
        save_params(path, merged, comment="test")
        loaded = load_params(path)
        assert abs(loaded["MC_ROLLRATE_P"] - 0.30) < 1e-6
        # Untouched parameter preserved from baseline
        assert "MC_PITCHRATE_P" in loaded
    finally:
        os.unlink(path)
```

- [ ] **Step 2: Run to verify they fail**

```bash
cd ulg_analysis
uv run --with pyulog,numpy,scipy,pandas,pytest pytest tests/test_params_io.py -v
```
Expected: `FAILED` — `ModuleNotFoundError`

- [ ] **Step 3: Implement params_io**

```python
# ulg_analysis/pid_optimizer/params_io.py
"""
Read and write PX4 .params files (QGroundControl format).

Line format (tab-separated):
  component_id  param_id  name  value  type
Example:
  1  1  MC_ROLLRATE_P  0.150000  9
"""
from datetime import date
from typing import Dict


def load_params(path: str) -> Dict[str, float]:
    """Parse a .params file and return {name: value} for all numeric params."""
    params = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split("\t")
            if len(parts) < 5:
                continue
            name = parts[2].strip()
            try:
                value = float(parts[3])
            except ValueError:
                continue
            params[name] = value
    return params


def save_params(path: str, params: Dict[str, float], comment: str = "") -> None:
    """Write {name: value} dict to a QGroundControl-compatible .params file."""
    lines = []
    if comment:
        lines.append(f"# {comment}")
    for name, value in sorted(params.items()):
        lines.append(f"1\t1\t{name}\t{value:.9f}\t9")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd ulg_analysis
uv run --with pyulog,numpy,scipy,pandas,pytest pytest tests/test_params_io.py -v
```
Expected: all 3 `PASSED`

- [ ] **Step 5: Commit**

```bash
git add ulg_analysis/pid_optimizer/params_io.py ulg_analysis/tests/test_params_io.py
git commit -m "feat: add params_io for reading and writing PX4 .params files"
```

---

## Task 7: Notebook 01 — ULG Explorer

**Files:**
- Create: `ulg_analysis/notebooks/01_ulg_explorer.ipynb`

- [ ] **Step 1: Create the notebook**

Create `ulg_analysis/notebooks/01_ulg_explorer.ipynb` with the following cells:

**Cell 1 — Imports**
```python
import pyulog
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

LOG_ROOT = Path("../px4_logs")
```

**Cell 2 — List all .ulg files and extract topic availability**
```python
ulg_files = sorted(LOG_ROOT.rglob("*.ulg"))
print(f"Found {len(ulg_files)} .ulg files")

rows = []
for p in ulg_files:
    try:
        log = pyulog.ULog(str(p), disable_str_exceptions=True)
        topics = {d.name for d in log.data_list}
        rows.append({"path": str(p.relative_to(LOG_ROOT)), "topics": len(topics),
                     "has_actuator": "actuator_outputs" in topics,
                     "has_imu": "sensor_combined" in topics,
                     "has_rates": "vehicle_angular_velocity" in topics,
                     "has_position": "vehicle_local_position" in topics,
                     "has_attitude": "vehicle_attitude" in topics,
                     "has_rate_sp": "vehicle_rates_setpoint" in topics,
                     "has_pos_sp": "vehicle_local_position_setpoint" in topics,
                     "duration_s": (log.last_timestamp - log.start_timestamp) / 1e6})
    except Exception as e:
        rows.append({"path": str(p.relative_to(LOG_ROOT)), "error": str(e)})

df = pd.DataFrame(rows)
df
```

**Cell 3 — Heatmap of signal availability**
```python
bool_cols = ["has_actuator", "has_imu", "has_rates", "has_position",
             "has_attitude", "has_rate_sp", "has_pos_sp"]
heat = df[bool_cols].astype(float)
fig, ax = plt.subplots(figsize=(10, max(4, len(heat) * 0.3)))
im = ax.imshow(heat.T, aspect="auto", cmap="RdYlGn", vmin=0, vmax=1)
ax.set_yticks(range(len(bool_cols))); ax.set_yticklabels(bool_cols)
ax.set_xlabel("Log index"); ax.set_title("Signal availability across all .ulg files")
plt.tight_layout(); plt.show()
```

**Cell 4 — Deep dive: plot key signals from a specific log**
```python
LOG_PATH = "../px4_logs/Hermit/testes_PID_position_31-08/log_12_2024-8-30-16-21-54.ulg"
log = pyulog.ULog(LOG_PATH, disable_str_exceptions=True)
t0 = log.start_timestamp

def to_df(log, topic, multi_id=0):
    d = next(d for d in log.data_list if d.name == topic and d.multi_id == multi_id)
    df = pd.DataFrame({f: d.data[f] for f in d.data})
    df["t_s"] = (df["timestamp"] - t0) / 1e6
    return df

fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)

# Actuator outputs
act = to_df(log, "actuator_outputs")
for i in range(4):
    axes[0].plot(act["t_s"], act[f"output[{i}]"], label=f"motor {i+1}")
axes[0].set_title("Actuator Outputs"); axes[0].legend()

# Angular velocity
rates = to_df(log, "vehicle_angular_velocity")
for ax_name, col in [("roll", "xyz[0]"), ("pitch", "xyz[1]"), ("yaw", "xyz[2]")]:
    axes[1].plot(rates["t_s"], np.rad2deg(rates[col]), label=ax_name)
axes[1].set_title("Angular Velocity (deg/s)"); axes[1].legend()

# Attitude
att = to_df(log, "vehicle_attitude")
# Quaternion → Euler
from scipy.spatial.transform import Rotation as R_
q = att[["q[0]", "q[1]", "q[2]", "q[3]"]].values  # [w, x, y, z]
euler = R_.from_quat(q[:, [1, 2, 3, 0]]).as_euler("xyz", degrees=True)
for i, name in enumerate(["roll", "pitch", "yaw"]):
    axes[2].plot(att["t_s"].values, euler[:, i], label=name)
axes[2].set_title("Attitude (deg)"); axes[2].legend()

# Local position
pos = to_df(log, "vehicle_local_position")
for col, name in [("x", "X"), ("y", "Y"), ("z", "Z")]:
    axes[3].plot(pos["t_s"], pos[col], label=name)
axes[3].set_title("Local Position (m)"); axes[3].legend(); axes[3].set_xlabel("Time (s)")

plt.tight_layout(); plt.show()
```

**Cell 5 — Print all initial PID parameters from the log**
```python
print("=== PID Parameters from log ===")
pid_params = {k: v for k, v in log.initial_parameters.items()
              if any(x in k for x in ["MC_ROLL", "MC_PITCH", "MC_YAW", "MPC_XY", "MPC_Z_"])}
for k in sorted(pid_params):
    print(f"  {k}: {pid_params[k]:.6f}")
```

- [ ] **Step 2: Run the notebook**

```bash
cd ulg_analysis
uv run --with pyulog,numpy,scipy,pandas,matplotlib,jupyter jupyter nbconvert \
  --to notebook --execute notebooks/01_ulg_explorer.ipynb \
  --output notebooks/01_ulg_explorer_executed.ipynb 2>&1 | tail -5
```
Expected: `Wrote ... notebooks/01_ulg_explorer_executed.ipynb`

All key signals should appear in the heatmap as green for the Hermit PID test log.

- [ ] **Step 3: Commit**

```bash
git add ulg_analysis/notebooks/01_ulg_explorer.ipynb
git commit -m "feat: add ULG explorer notebook with signal heatmap and key plots"
```

---

## Task 8: Notebook 02 — System Identification

**Files:**
- Create: `ulg_analysis/notebooks/02_sysid.ipynb`
- Create: `ulg_analysis/artifacts/.gitkeep`

- [ ] **Step 1: Create the notebook**

Create `ulg_analysis/notebooks/02_sysid.ipynb` with the following cells:

**Cell 1 — Setup**
```python
import sys; sys.path.insert(0, "..")
import pyulog
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import least_squares, curve_fit
from pathlib import Path
from pid_optimizer.plant_model import PlantModel, Inertia, Drag

LOG_PATH = "../px4_logs/Hermit/testes_PID_position_31-08/log_12_2024-8-30-16-21-54.ulg"
ARTIFACT_PATH = "../artifacts/plant_model.json"
Path("../artifacts").mkdir(exist_ok=True)

log = pyulog.ULog(LOG_PATH, disable_str_exceptions=True)
t0 = log.start_timestamp

def to_df(log, topic, multi_id=0):
    d = next(d for d in log.data_list if d.name == topic and d.multi_id == multi_id)
    df = pd.DataFrame({f: d.data[f] for f in d.data})
    df["t_s"] = (df["timestamp"] - t0) / 1e6
    return df.sort_values("t_s").reset_index(drop=True)
```

**Cell 2 — Thrust coefficient from hover**
```python
# During hover, F_total = mass * g → kT * sum(cmd_i^2) = mass * g
# Identify hover segments: |vz| < 0.1 m/s, |roll|<5°, |pitch|<5° for >2s

vel = to_df(log, "vehicle_local_position")
att = to_df(log, "vehicle_attitude")
act = to_df(log, "actuator_outputs")

# Align timestamps by interpolation
t_ref = act["t_s"].values
vz_interp = np.interp(t_ref, vel["t_s"].values, vel["vz"].values)

# Hover segments: vz near 0
hover_mask = np.abs(vz_interp) < 0.15
print(f"Hover fraction: {hover_mask.mean():.1%}")

hover_act = act[hover_mask]
# Normalize PWM: PX4 outputs typically in range [1000, 2000] µs; normalize to [0, 1]
pwm_cols = [c for c in act.columns if c.startswith("output[")][:4]
cmds = hover_act[pwm_cols].values
cmd_min, cmd_max = 1000.0, 2000.0
cmds_norm = np.clip((cmds - cmd_min) / (cmd_max - cmd_min), 0.0, 1.0)

# kT * sum(cmd_i^2) = mass * g  →  kT = mass * g / mean(sum(cmd_i^2))
MASS_KG = 1.8  # drone mass — update this to your measured value!
G = 9.81
sum_sq = np.mean(np.sum(cmds_norm ** 2, axis=1))
kT = MASS_KG * G / sum_sq
print(f"Estimated kT = {kT:.6f} N per (normalized_cmd^2) per motor")
```

**Cell 3 — Moment of inertia via least squares**
```python
# τ = I × α  →  I = τ / α
# Differential thrust → torque; gyro derivative → angular acceleration

rates_df = to_df(log, "vehicle_angular_velocity")
t_rate = rates_df["t_s"].values
p_rate = np.interp(t_ref, t_rate, rates_df["xyz[0]"].values)
q_rate = np.interp(t_ref, t_rate, rates_df["xyz[1]"].values)
r_rate = np.interp(t_ref, t_rate, rates_df["xyz[2]"].values)

dt = np.diff(t_ref)
alpha_roll  = np.gradient(p_rate, t_ref)
alpha_pitch = np.gradient(q_rate, t_ref)
alpha_yaw   = np.gradient(r_rate, t_ref)

cmds_all = act[pwm_cols].values
cmds_all_norm = np.clip((cmds_all - cmd_min) / (cmd_max - cmd_min), 0.0, 1.0)
arm = 0.17  # arm length meters — update to your measured value!

# X-frame torque model:  τ_roll = kT*arm*(−m1 + m2 + m3 − m4)  (sign depends on layout)
tau_roll  = kT * arm * (-cmds_all_norm[:,0] + cmds_all_norm[:,1]
                        + cmds_all_norm[:,2] - cmds_all_norm[:,3])
tau_pitch = kT * arm * (-cmds_all_norm[:,0] - cmds_all_norm[:,1]
                        + cmds_all_norm[:,2] + cmds_all_norm[:,3])
# Yaw uses reaction torque coefficient (≈ 0.1 * kT * arm for typical props)
tau_yaw   = 0.1 * kT * arm * (-cmds_all_norm[:,0] + cmds_all_norm[:,1]
                               - cmds_all_norm[:,2] + cmds_all_norm[:,3])

# Least squares: I = mean(tau / alpha)  (exclude near-zero alpha)
mask_roll  = np.abs(alpha_roll)  > 0.5
mask_pitch = np.abs(alpha_pitch) > 0.5
mask_yaw   = np.abs(alpha_yaw)   > 0.1
Ixx = np.median(tau_roll[mask_roll]  / alpha_roll[mask_roll])   if mask_roll.any()  else 0.012
Iyy = np.median(tau_pitch[mask_pitch]/ alpha_pitch[mask_pitch]) if mask_pitch.any() else 0.013
Izz = np.median(tau_yaw[mask_yaw]   / alpha_yaw[mask_yaw])     if mask_yaw.any()   else 0.022
print(f"Ixx={Ixx:.5f}  Iyy={Iyy:.5f}  Izz={Izz:.5f}  kg·m²")
```

**Cell 4 — Motor time constant via first-order lag fit**
```python
# Find a large throttle step-up, fit: y(t) = y_final*(1 - exp(-t/tau))
from scipy.optimize import curve_fit

# Look at motor 0 command signal
m0 = act[pwm_cols[0]].values
m0_norm = np.clip((m0 - cmd_min) / (cmd_max - cmd_min), 0.0, 1.0)
steps = np.where(np.diff(m0_norm) > 0.1)[0]

tau_motor = 0.03  # default if no clear step found
if len(steps) > 0:
    i0 = steps[0]
    seg_t = t_ref[i0:i0+100] - t_ref[i0]
    seg_y = m0_norm[i0:i0+100]
    y_final = seg_y[-1]
    def first_order(t, tau): return y_final * (1 - np.exp(-t / tau))
    try:
        popt, _ = curve_fit(first_order, seg_t, seg_y, p0=[0.03], bounds=(0.005, 0.2))
        tau_motor = float(popt[0])
    except Exception:
        pass
print(f"Motor time constant τ = {tau_motor*1000:.1f} ms")
```

**Cell 5 — Aerodynamic drag from velocity decay**
```python
# Find segments where thrust ≈ 0 (motors at minimum) and drone is gliding
thrust_total = kT * np.sum(cmds_all_norm**2, axis=1)
glide_mask = thrust_total < (0.1 * MASS_KG * G)
vx_interp = np.interp(t_ref, vel["t_s"].values, vel["vx"].values)
vy_interp = np.interp(t_ref, vel["t_s"].values, vel["vy"].values)

# kD_xy: a_drag = -kD_xy/mass * v → fit exponential decay
kD_xy, kD_z = 0.15, 0.20  # defaults
if glide_mask.sum() > 50:
    v_xy = np.sqrt(vx_interp[glide_mask]**2 + vy_interp[glide_mask]**2)
    a_xy = np.gradient(v_xy, t_ref[glide_mask])
    valid = v_xy > 0.1
    if valid.sum() > 20:
        kD_xy = float(np.median(-MASS_KG * a_xy[valid] / v_xy[valid]))
        kD_xy = np.clip(kD_xy, 0.01, 2.0)
print(f"kD_xy = {kD_xy:.4f}  kD_z = {kD_z:.4f}")
```

**Cell 6 — Assemble and save plant model**
```python
model = PlantModel(
    mass_kg=MASS_KG, kT=kT, tau_motor_s=tau_motor,
    inertia=Inertia(Ixx=float(Ixx), Iyy=float(Iyy), Izz=float(Izz)),
    drag=Drag(kD_xy=float(kD_xy), kD_z=float(kD_z)),
    arm_length_m=arm,
    source_log=LOG_PATH,
    fit_rmse={},
)
model.save(ARTIFACT_PATH)
print(f"Saved to {ARTIFACT_PATH}")
import json; print(json.dumps(model.__dict__ if not hasattr(model, '__dataclass_fields__') else
    {k: getattr(model, k) for k in model.__dataclass_fields__}, default=str, indent=2))
```

**Cell 7 — Validation: replay motor commands through model**
```python
# Forward simulate angular rates using identified model and compare to log
from pid_optimizer.plant_model import PlantModel
m = PlantModel.load(ARTIFACT_PATH)

I_vec = np.array([m.inertia.Ixx, m.inertia.Iyy, m.inertia.Izz])
sim_omega = np.zeros(3)
sim_omegas = []
for i in range(len(t_ref) - 1):
    dt_i = t_ref[i+1] - t_ref[i]
    tau = np.array([tau_roll[i], tau_pitch[i], tau_yaw[i]])
    alpha = (tau - np.cross(sim_omega, I_vec * sim_omega)) / I_vec
    alpha *= dt_i / (m.tau_motor_s + dt_i)  # motor lag
    sim_omega = sim_omega + alpha * dt_i
    sim_omegas.append(sim_omega.copy())

sim_omegas = np.array(sim_omegas)
fig, axes = plt.subplots(3, 1, figsize=(14, 8), sharex=True)
labels = ["Roll rate (rad/s)", "Pitch rate (rad/s)", "Yaw rate (rad/s)"]
for i, label in enumerate(labels):
    axes[i].plot(t_ref[1:], sim_omegas[:, i], label="simulated", alpha=0.8)
    axes[i].plot(t_ref, [p_rate, q_rate, r_rate][i], label="real", alpha=0.6)
    axes[i].set_ylabel(label); axes[i].legend()
axes[0].set_title("Model validation: simulated vs real angular rates")
axes[-1].set_xlabel("Time (s)")
plt.tight_layout(); plt.show()

rmse_roll  = np.sqrt(np.mean((sim_omegas[:, 0] - p_rate[1:])**2))
rmse_pitch = np.sqrt(np.mean((sim_omegas[:, 1] - q_rate[1:])**2))
print(f"RMSE — roll: {rmse_roll:.4f} rad/s, pitch: {rmse_pitch:.4f} rad/s")
print("Target: RMSE < 10% of signal range")
print(f"Roll range: {p_rate.ptp():.4f}, 10% = {p_rate.ptp()*0.1:.4f}")
```

- [ ] **Step 2: Create artifacts dir placeholder**

```bash
mkdir -p ulg_analysis/artifacts && touch ulg_analysis/artifacts/.gitkeep
echo "artifacts/" >> ulg_analysis/.gitignore
```

- [ ] **Step 3: Commit**

```bash
git add ulg_analysis/notebooks/02_sysid.ipynb ulg_analysis/artifacts/.gitkeep
git commit -m "feat: add system identification notebook (thrust, inertia, motor lag, drag)"
```

---

## Task 9: Notebook 03 — PX4 Simulator Validation

**Files:**
- Create: `ulg_analysis/notebooks/03_px4_simulator.ipynb`

- [ ] **Step 1: Create the notebook**

Create `ulg_analysis/notebooks/03_px4_simulator.ipynb` with the following cells:

**Cell 1 — Setup**
```python
import sys; sys.path.insert(0, "..")
import pyulog
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R_
from pid_optimizer.plant_model import PlantModel
from pid_optimizer.gains import Gains, HERMIT_REFERENCE_GAINS
from pid_optimizer.pid_simulator import PX4Simulator, compute_rmse

LOG_PATH = "../px4_logs/Hermit/testes_PID_position_31-08/log_12_2024-8-30-16-21-54.ulg"
PLANT_PATH = "../artifacts/plant_model.json"

plant = PlantModel.load(PLANT_PATH)
gains = HERMIT_REFERENCE_GAINS
log = pyulog.ULog(LOG_PATH, disable_str_exceptions=True)
t0 = log.start_timestamp

def to_df(log, topic, multi_id=0):
    d = next(d for d in log.data_list if d.name == topic and d.multi_id == multi_id)
    df = pd.DataFrame({f: d.data[f] for f in d.data})
    df["t_s"] = (df["timestamp"] - t0) / 1e6
    return df.sort_values("t_s").reset_index(drop=True)
```

**Cell 2 — Build setpoints DataFrame from log**
```python
DT = 0.004  # 250 Hz simulation rate
pos_sp_df = to_df(log, "vehicle_local_position_setpoint")
att_sp_df = to_df(log, "vehicle_attitude_setpoint")
pos_df    = to_df(log, "vehicle_local_position")
att_df    = to_df(log, "vehicle_attitude")

# Use first 30 seconds of the log
T_MAX = 30.0
t_sim = np.arange(0, T_MAX, DT)

def interp_col(src_df, col, t_out):
    return np.interp(t_out, src_df["t_s"].values, src_df[col].values)

# Convert attitude setpoint quaternion → yaw setpoint
q_sp = att_sp_df[["q_d[0]", "q_d[1]", "q_d[2]", "q_d[3]"]].values  # w,x,y,z
yaw_sp = R_.from_quat(q_sp[:, [1,2,3,0]]).as_euler("xyz")[:, 2]
yaw_sp_interp = np.interp(t_sim, att_sp_df["t_s"].values, yaw_sp)

setpoints = pd.DataFrame({
    "x":    interp_col(pos_sp_df, "x",   t_sim),
    "y":    interp_col(pos_sp_df, "y",   t_sim),
    "z":    interp_col(pos_sp_df, "z",   t_sim),
    "vx":   interp_col(pos_sp_df, "vx",  t_sim),
    "vy":   interp_col(pos_sp_df, "vy",  t_sim),
    "vz":   interp_col(pos_sp_df, "vz",  t_sim),
    "roll": [0.0] * len(t_sim),
    "pitch":[0.0] * len(t_sim),
    "yaw":  yaw_sp_interp,
})
print(f"Setpoints shape: {setpoints.shape}")
```

**Cell 3 — Run simulator and plot results**
```python
sim = PX4Simulator(plant, gains)
result = sim.run(setpoints, dt=DT)

# Real state for comparison
q_real = att_df[["q[0]","q[1]","q[2]","q[3]"]].values  # w,x,y,z
euler_real = R_.from_quat(q_real[:, [1,2,3,0]]).as_euler("xyz")
t_real = att_df["t_s"].values

fig, axes = plt.subplots(3, 2, figsize=(16, 12))
t_plot = t_sim[:len(result)]

# Position
for i, (col, label) in enumerate([("x","X(m)"), ("y","Y(m)"), ("z","Z(m)")]):
    ax = axes[i, 0]
    ax.plot(t_plot, result[col].values, label="simulated")
    ax.plot(pos_df["t_s"].values, pos_df[col].values, label="real", alpha=0.6)
    ax.set_ylabel(label); ax.legend(); ax.set_title(f"Position {label}")

# Attitude
for i, (idx, label) in enumerate([(0,"Roll(deg)"), (1,"Pitch(deg)"), (2,"Yaw(deg)")]):
    ax = axes[i, 1]
    ax.plot(t_plot, np.rad2deg(result[["roll","pitch","yaw"]].values[:, idx]), label="simulated")
    ax.plot(t_real[t_real<T_MAX], np.rad2deg(euler_real[t_real<T_MAX, idx]), label="real", alpha=0.6)
    ax.set_ylabel(label); ax.legend(); ax.set_title(f"Attitude {label}")

axes[-1, 0].set_xlabel("Time (s)")
axes[-1, 1].set_xlabel("Time (s)")
plt.suptitle("Simulator validation: simulated vs real state", fontsize=13)
plt.tight_layout(); plt.show()
```

**Cell 4 — RMSE report**
```python
# Build reference DataFrame at simulation timestamps
ref = pd.DataFrame({
    "x": interp_col(pos_df, "x", t_sim),
    "y": interp_col(pos_df, "y", t_sim),
    "z": interp_col(pos_df, "z", t_sim),
})
rmse = compute_rmse(result[["x","y","z"]], ref[["x","y","z"]], cols=["x","y","z"])
print("Position RMSE:")
for col, val in rmse.items():
    sig_range = ref[col].max() - ref[col].min()
    pct = 100*val/sig_range if sig_range > 0.1 else float('nan')
    status = "OK" if pct < 15 else "WARN"
    print(f"  {col}: {val:.3f} m  ({pct:.1f}% of range)  [{status}]")
```

- [ ] **Step 2: Run the notebook**

```bash
cd ulg_analysis
uv run --with pyulog,numpy,scipy,pandas,matplotlib,jupyter jupyter nbconvert \
  --to notebook --execute notebooks/03_px4_simulator.ipynb \
  --output notebooks/03_px4_simulator_executed.ipynb 2>&1 | tail -5
```
Expected: Notebook executes. RMSE values printed. Plots visible in output.

- [ ] **Step 3: Commit**

```bash
git add ulg_analysis/notebooks/03_px4_simulator.ipynb
git commit -m "feat: add PX4 simulator validation notebook"
```

---

## Task 10: Notebook 04 — Genetic Algorithm Optimizer

**Files:**
- Create: `ulg_analysis/notebooks/04_ga_optimizer.ipynb`

- [ ] **Step 1: Create the notebook**

Create `ulg_analysis/notebooks/04_ga_optimizer.ipynb` with the following cells:

**Cell 1 — Setup**
```python
import sys; sys.path.insert(0, "..")
import pyulog
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R_
from pathlib import Path
from pid_optimizer.plant_model import PlantModel
from pid_optimizer.gains import Gains, GAIN_BOUNDS, HERMIT_REFERENCE_GAINS
from pid_optimizer.pid_simulator import PX4Simulator, compute_rmse
from pid_optimizer.ga import GeneticAlgorithm

PLANT_PATH   = "../artifacts/plant_model.json"
CHECKPOINT   = "../artifacts/ga_checkpoint.json"
BEST_GAINS   = "../artifacts/best_gains.json"
LOG_PATH     = "../px4_logs/Hermit/testes_PID_position_31-08/log_12_2024-8-30-16-21-54.ulg"

plant = PlantModel.load(PLANT_PATH)
```

**Cell 2 — Build reference setpoints and actual trajectory (reuse from nb03)**
```python
log = pyulog.ULog(LOG_PATH, disable_str_exceptions=True)
t0 = log.start_timestamp

def to_df(log, topic, multi_id=0):
    d = next(d for d in log.data_list if d.name == topic and d.multi_id == multi_id)
    df = pd.DataFrame({f: d.data[f] for f in d.data})
    df["t_s"] = (df["timestamp"] - t0) / 1e6
    return df.sort_values("t_s").reset_index(drop=True)

DT, T_MAX = 0.004, 20.0
t_sim = np.arange(0, T_MAX, DT)

pos_sp_df = to_df(log, "vehicle_local_position_setpoint")
att_sp_df = to_df(log, "vehicle_attitude_setpoint")
pos_df    = to_df(log, "vehicle_local_position")

def interp_col(src_df, col, t_out):
    return np.interp(t_out, src_df["t_s"].values, src_df[col].values)

q_sp = att_sp_df[["q_d[0]","q_d[1]","q_d[2]","q_d[3]"]].values
yaw_sp = R_.from_quat(q_sp[:, [1,2,3,0]]).as_euler("xyz")[:, 2]

setpoints = pd.DataFrame({
    "x": interp_col(pos_sp_df, "x", t_sim), "y": interp_col(pos_sp_df, "y", t_sim),
    "z": interp_col(pos_sp_df, "z", t_sim), "vx": interp_col(pos_sp_df, "vx", t_sim),
    "vy": interp_col(pos_sp_df, "vy", t_sim), "vz": interp_col(pos_sp_df, "vz", t_sim),
    "roll": 0.0, "pitch": 0.0,
    "yaw": np.interp(t_sim, att_sp_df["t_s"].values, yaw_sp),
})

actual_pos = pd.DataFrame({
    "x": interp_col(pos_df, "x", t_sim),
    "y": interp_col(pos_df, "y", t_sim),
    "z": interp_col(pos_df, "z", t_sim),
})

# Baseline RMSE with reference gains
ref_sim = PX4Simulator(plant, HERMIT_REFERENCE_GAINS).run(setpoints, dt=DT)
baseline_rmse = compute_rmse(ref_sim[["x","y","z"]], actual_pos, cols=["x","y","z"])
print("Baseline RMSE (reference gains):", {k: f"{v:.4f}" for k, v in baseline_rmse.items()})
```

**Cell 3 — Define fitness function**
```python
# Fitness weights: inner loops count more
WEIGHTS = {"roll": 4.0, "pitch": 4.0, "yaw": 3.0, "x": 1.5, "y": 1.5, "z": 2.0}

def fitness(gains_array: np.ndarray) -> float:
    g = Gains.from_array(gains_array)
    try:
        result = PX4Simulator(plant, g).run(setpoints, dt=DT)
        rmse = compute_rmse(result[["x","y","z"]], actual_pos, cols=["x","y","z"])
        score = (WEIGHTS["x"] * rmse["x"] + WEIGHTS["y"] * rmse["y"] + WEIGHTS["z"] * rmse["z"])
        if not np.isfinite(score): return 1e6
        return float(score)
    except Exception:
        return 1e6

# Quick check
ref_score = fitness(HERMIT_REFERENCE_GAINS.to_array())
print(f"Reference gains fitness: {ref_score:.4f}")
```

**Cell 4 — Run GA (or resume from checkpoint)**
```python
import os, json

GENERATIONS = 200
CHECKPOINT_EVERY = 10

if os.path.exists(CHECKPOINT):
    ga = GeneticAlgorithm(GAIN_BOUNDS, fitness_fn=fitness, pop_size=100, seed=42)
    ga.load_checkpoint(CHECKPOINT)
    print(f"Resumed from generation {ga.generation}, best fitness={ga.best_fitness:.4f}")
else:
    ga = GeneticAlgorithm(GAIN_BOUNDS, fitness_fn=fitness, pop_size=100, seed=42)
    ga.initialize()
    print(f"Initialized. Generation 0 best fitness={ga.best_fitness:.4f}")

remaining = GENERATIONS - ga.generation
print(f"Running {remaining} more generations...")
for gen in range(remaining):
    ga.step()
    if ga.generation % CHECKPOINT_EVERY == 0:
        ga.save_checkpoint(CHECKPOINT)
        print(f"Gen {ga.generation:4d} | best={ga.best_fitness:.4f} | mean={ga.history[-1]['mean']:.4f}")

# Save best N candidates
scores_argsort = ga.scores.argsort()
best_n = [{"gains": ga.population[i].tolist(), "fitness": float(ga.scores[i])}
          for i in scores_argsort[:5]]
with open(BEST_GAINS, "w") as f:
    json.dump({"reference_fitness": ref_score, "candidates": best_n}, f, indent=2)
print(f"\nBest fitness: {ga.best_fitness:.4f}  (reference: {ref_score:.4f})")
print(f"Saved to {BEST_GAINS}")
```

**Cell 5 — Convergence plot**
```python
gens    = [h["generation"] for h in ga.history]
bests   = [h["best"]  for h in ga.history]
means   = [h["mean"]  for h in ga.history]
stds    = [h["std"]   for h in ga.history]

fig, ax = plt.subplots(figsize=(12, 4))
ax.plot(gens, bests, label="best", linewidth=2)
ax.plot(gens, means, label="mean", alpha=0.7)
ax.fill_between(gens, np.array(means)-np.array(stds),
                np.array(means)+np.array(stds), alpha=0.2, label="±1σ")
ax.axhline(ref_score, color="red", linestyle="--", label="reference")
ax.set_xlabel("Generation"); ax.set_ylabel("Fitness (weighted RMSE)")
ax.set_title("GA Convergence"); ax.legend()
plt.tight_layout(); plt.show()
```

**Cell 6 — Parameter heat map**
```python
from pid_optimizer.gains import Gains
from dataclasses import fields

gain_names = [f.name for f in fields(Gains)]
lo = np.array([b[0] for b in GAIN_BOUNDS])
hi = np.array([b[1] for b in GAIN_BOUNDS])
top10 = ga.population[ga.scores.argsort()[:10]]
normalized = (top10 - lo) / (hi - lo + 1e-12)

fig, ax = plt.subplots(figsize=(14, 4))
im = ax.imshow(normalized.T, aspect="auto", cmap="coolwarm", vmin=0, vmax=1)
ax.set_yticks(range(len(gain_names))); ax.set_yticklabels(gain_names, fontsize=8)
ax.set_xlabel("Top-10 individuals"); ax.set_title("Gain values (normalized to bounds)")
plt.colorbar(im, ax=ax, label="0=low bound, 1=high bound")
plt.tight_layout(); plt.show()
```

- [ ] **Step 2: Commit**

```bash
git add ulg_analysis/notebooks/04_ga_optimizer.ipynb
git commit -m "feat: add GA optimizer notebook with fitness function and convergence plots"
```

---

## Task 11: Notebook 05 — Params Export

**Files:**
- Create: `ulg_analysis/notebooks/05_params_export.ipynb`

- [ ] **Step 1: Create the notebook**

Create `ulg_analysis/notebooks/05_params_export.ipynb` with the following cells:

**Cell 1 — Setup**
```python
import sys; sys.path.insert(0, "..")
import json, os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from datetime import date
from pid_optimizer.gains import Gains, HERMIT_REFERENCE_GAINS, GAIN_BOUNDS
from pid_optimizer.plant_model import PlantModel
from pid_optimizer.pid_simulator import PX4Simulator
from pid_optimizer.params_io import load_params, save_params

BEST_GAINS_PATH  = "../artifacts/best_gains.json"
BASELINE_PARAMS  = "../px4_logs/pixhawk-6X/dia_20-06/parametersV6x6_20.params"
PLANT_PATH       = "../artifacts/plant_model.json"

with open(BEST_GAINS_PATH) as f:
    best = json.load(f)

candidates = best["candidates"]
ref_fitness = best["reference_fitness"]
print(f"Reference fitness: {ref_fitness:.4f}")
print(f"Top candidates:")
for i, c in enumerate(candidates):
    print(f"  #{i+1}  fitness={c['fitness']:.4f}  (Δ={c['fitness']-ref_fitness:+.4f})")
```

**Cell 2 — Comparison table: reference vs best candidate**
```python
from dataclasses import fields

gain_names = [f.name for f in fields(Gains)]
ref_arr    = HERMIT_REFERENCE_GAINS.to_array()
best_arr   = np.array(candidates[0]["gains"])

rows = []
for name, ref_val, opt_val, bounds in zip(gain_names, ref_arr, best_arr, GAIN_BOUNDS):
    pct_change = 100 * (opt_val - ref_val) / (ref_val + 1e-12)
    flag = "⚠️ >50%" if abs(pct_change) > 50 else ""
    rows.append({"parameter": name, "reference": f"{ref_val:.6f}",
                  "optimized": f"{opt_val:.6f}", "change_%": f"{pct_change:+.1f}", "flag": flag})

df = pd.DataFrame(rows)
# Show flagged rows first
df_sorted = pd.concat([df[df["flag"] != ""], df[df["flag"] == ""]])
print(df_sorted.to_string(index=False))
```

**Cell 3 — Step response comparison: reference vs optimized**
```python
import pyulog
from scipy.spatial.transform import Rotation as R_

LOG_PATH = "../px4_logs/Hermit/testes_PID_position_31-08/log_12_2024-8-30-16-21-54.ulg"
plant = PlantModel.load(PLANT_PATH)
log = pyulog.ULog(LOG_PATH, disable_str_exceptions=True)
t0 = log.start_timestamp

def to_df(log, topic, multi_id=0):
    d = next(d for d in log.data_list if d.name == topic and d.multi_id == multi_id)
    df = pd.DataFrame({f: d.data[f] for f in d.data})
    df["t_s"] = (df["timestamp"] - t0) / 1e6
    return df.sort_values("t_s").reset_index(drop=True)

DT, T_MAX = 0.004, 20.0
t_sim = np.arange(0, T_MAX, DT)

pos_sp_df = to_df(log, "vehicle_local_position_setpoint")
att_sp_df = to_df(log, "vehicle_attitude_setpoint")
q_sp = att_sp_df[["q_d[0]","q_d[1]","q_d[2]","q_d[3]"]].values
yaw_sp = R_.from_quat(q_sp[:, [1,2,3,0]]).as_euler("xyz")[:, 2]

def interp_col(src_df, col, t_out):
    return np.interp(t_out, src_df["t_s"].values, src_df[col].values)

setpoints = pd.DataFrame({
    "x": interp_col(pos_sp_df, "x", t_sim), "y": interp_col(pos_sp_df, "y", t_sim),
    "z": interp_col(pos_sp_df, "z", t_sim), "vx": interp_col(pos_sp_df, "vx", t_sim),
    "vy": interp_col(pos_sp_df, "vy", t_sim), "vz": interp_col(pos_sp_df, "vz", t_sim),
    "roll": 0.0, "pitch": 0.0,
    "yaw": np.interp(t_sim, att_sp_df["t_s"].values, yaw_sp),
})

ref_gains = HERMIT_REFERENCE_GAINS
opt_gains = Gains.from_array(np.array(candidates[0]["gains"]))

res_ref = PX4Simulator(plant, ref_gains).run(setpoints, dt=DT)
res_opt = PX4Simulator(plant, opt_gains).run(setpoints, dt=DT)

fig, axes = plt.subplots(3, 1, figsize=(14, 8), sharex=True)
for i, (col, label) in enumerate([("x","X(m)"),("y","Y(m)"),("z","Z(m)")]):
    axes[i].plot(t_sim[:len(res_ref)], res_ref[col].values, label="reference gains", alpha=0.8)
    axes[i].plot(t_sim[:len(res_opt)], res_opt[col].values, label="optimized gains", alpha=0.8)
    axes[i].plot(t_sim, setpoints[col].values, "k--", label="setpoint", alpha=0.4)
    axes[i].set_ylabel(label); axes[i].legend()
axes[-1].set_xlabel("Time (s)")
plt.suptitle("Simulated trajectory: reference vs optimized gains")
plt.tight_layout(); plt.show()
```

**Cell 4 — Choose candidate and export .params**
```python
CANDIDATE_IDX = 0  # Change to 1, 2, etc. to pick a different candidate
chosen_gains = Gains.from_array(np.array(candidates[CANDIDATE_IDX]["gains"]))
chosen_fitness = candidates[CANDIDATE_IDX]["fitness"]

# Load baseline params (all non-tuned parameters come from here)
baseline = load_params(BASELINE_PARAMS)

# Merge: tuned parameters overwrite baseline
optimized_params = {**baseline, **chosen_gains.to_px4_params()}

# Export
today = date.today().isoformat()
out_path = f"../artifacts/optimized_hermit_{today}.params"
comment = (f"PID Optimizer — Hermit — {today} — "
           f"fitness RMSE: {chosen_fitness:.4f} (ref: {ref_fitness:.4f})")
save_params(out_path, optimized_params, comment=comment)
print(f"Exported to: {out_path}")
print(f"Comment: {comment}")
```

- [ ] **Step 2: Commit**

```bash
git add ulg_analysis/notebooks/05_params_export.ipynb
git commit -m "feat: add params export notebook with comparison table and .params writer"
```

---

## Task 12: Run Full Test Suite

- [ ] **Step 1: Run all tests**

```bash
cd ulg_analysis
uv run --with pyulog,numpy,scipy,pandas,pytest pytest tests/ -v
```
Expected output:
```
tests/test_plant_model.py::test_roundtrip_json PASSED
tests/test_gains.py::test_to_array_from_array_roundtrip PASSED
tests/test_gains.py::test_array_length PASSED
tests/test_gains.py::test_bounds_length PASSED
tests/test_gains.py::test_to_px4_params_keys PASSED
tests/test_pid_simulator.py::test_rate_controller_converges_to_setpoint PASSED
tests/test_pid_simulator.py::test_hover_stays_near_origin PASSED
tests/test_pid_simulator.py::test_compute_rmse PASSED
tests/test_ga.py::test_tournament_selects_better PASSED
tests/test_ga.py::test_crossover_genes_from_parents PASSED
tests/test_ga.py::test_mutation_stays_in_bounds PASSED
tests/test_ga.py::test_ga_improves_fitness PASSED
tests/test_params_io.py::test_load_params_reads_known_value PASSED
tests/test_params_io.py::test_save_params_roundtrip PASSED
tests/test_params_io.py::test_save_params_only_updates_tuned_keys PASSED
================== 15 passed in <30s ==================
```

- [ ] **Step 2: Final commit**

```bash
git add -A
git commit -m "feat: complete PID optimizer pipeline — all tests passing"
```

---

## Spec Coverage Check

| Spec requirement | Task |
|---|---|
| 5 notebooks in `ulg_analysis/notebooks/` | Tasks 7–11 |
| ULG Explorer with heatmap | Task 7 |
| SysID: kT, inertia, motor lag, drag | Task 8 |
| Save `plant_model.json` | Task 8 |
| PX4 cascade: position→velocity→attitude→rate | Task 4 |
| Validation against real flight data | Task 9 |
| GA with tournament/crossover/mutation/elitism | Task 5 |
| 23 gain parameters with verified bounds | Task 3 |
| Checkpoint every 10 generations | Task 10 |
| Params export with comparison table | Task 11 |
| `>50%` change warning | Task 11 |
| `artifacts/` gitignored | Task 1 |
| `pyulog, numpy, scipy, pandas, matplotlib, jupyter` | Task 1 |
