from typing import Optional
import numpy as np
import pandas as pd
from .plant_model import PlantModel
from .gains import Gains


def _euler_to_R(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX rotation matrix: body to world."""
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
        return self.P * error + self.I * self._integral + self.D * d_term


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

    def run(self, setpoints: pd.DataFrame, dt: float = 0.004,
            x0: Optional[np.ndarray] = None) -> pd.DataFrame:
        """
        setpoints: DataFrame with columns [x, y, z, vx, vy, vz, roll, pitch, yaw]
        x0: optional initial state [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r].
            When None the simulator starts from all-zeros (wrong for real logs — always
            extract x0 from the ULG at t_start to avoid large initial attitude error).
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
        state = np.zeros(12) if x0 is None else x0.copy()
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

            # Acceleration setpoint → thrust + attitude (NED frame)
            # In NED: thrust opposes gravity (+z). Net acc_z = -thrust/m + g
            # So thrust = (G - acc_sp_z) * mass (more thrust to go up = negative acc_sp_z)
            thrust_sp = np.clip((G - acc_sp[2]) * p.mass_kg, 0.0, max_thrust)
            # The setpoint DataFrame may carry explicit roll/pitch attitude commands
            # (e.g. from a ULG-extracted attitude setpoint). When those are provided,
            # use them directly. When they are zero the acceleration-derived tilt from
            # lateral position/velocity control takes effect additively.
            roll_sp_from_acc  = np.clip(np.arctan2(acc_sp[1], G), -np.deg2rad(45), np.deg2rad(45))
            pitch_sp_from_acc = np.clip(np.arctan2(-acc_sp[0], G), -np.deg2rad(45), np.deg2rad(45))
            # NaN in roll/pitch column means "derive from acceleration"; explicit value overrides.
            roll_sp  = sp["roll"]  if not np.isnan(sp["roll"])  else roll_sp_from_acc
            pitch_sp = sp["pitch"] if not np.isnan(sp["pitch"]) else pitch_sp_from_acc
            att_sp = np.array([roll_sp, pitch_sp, sp["yaw"]])

            rate_sp = att_ctrl.update(att_sp, euler)
            torque_cmd = rate_ctrl.update(rate_sp, omega)

            # Rigid body dynamics (NED frame: z positive down, gravity = +G in z)
            R = _euler_to_R(*euler)
            # Thrust acts in -z_body direction (upward in body frame)
            thrust_body = np.array([0.0, 0.0, -thrust_sp])
            # a_world = thrust_world/m + gravity; gravity = [0, 0, +G] in NED
            a_world = R @ thrust_body / p.mass_kg + np.array([0.0, 0.0, G])
            drag_coeff = np.array([p.drag.kD_xy, p.drag.kD_xy, p.drag.kD_z])
            a_drag = -drag_coeff * vel / p.mass_kg
            a_total = a_world + a_drag

            # Rotational dynamics
            # Deliberate approximation: maps normalized rate-controller output [-1,1] to N·m.
            # Factors (50 roll/pitch, 20 yaw) are not derived from motor geometry — they
            # scale the dimensionless torque command to a physically plausible range given
            # the identified inertia. Replace with kT*arm*RPM² model after motor ID.
            scale = np.array([p.inertia.Ixx * 50, p.inertia.Iyy * 50, p.inertia.Izz * 20])
            tau = torque_cmd * scale
            alpha = (tau - np.cross(omega, I_vec * omega)) / I_vec

            # Motor lag
            alpha *= dt / (p.tau_motor_s + dt)

            # Integrate
            state[0:3] = pos + vel * dt
            state[3:6] = vel + a_total * dt
            state[6:9] = euler + omega * dt
            state[9:12] = omega + alpha * dt

            records.append(state.copy())

        cols = ["x", "y", "z", "vx", "vy", "vz", "roll", "pitch", "yaw", "p", "q", "r"]
        return pd.DataFrame(records, columns=cols)  # type: ignore[arg-type]


    def run_open_loop(
        self,
        torque_Nm: pd.DataFrame,
        thrust_N: pd.DataFrame,
        dt: float = 0.004,
        x0: Optional[np.ndarray] = None,
    ) -> pd.DataFrame:
        """
        Open-loop plant simulation — bypasses all PID controllers.

        torque_Nm: DataFrame with columns [tau_x, tau_y, tau_z] in N·m (FRD body frame).
                   Extracted from vehicle_torque_setpoint.xyz[0..2].
        thrust_N:  DataFrame with column [F] in Newtons, positive = upward.
                   Derived from vehicle_thrust_setpoint.xyz[2]:
                   F = plant.kT * 4 * xyz[2]**2
        x0: initial state [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r].
            Always extract from the ULG at t_start.

        Returns DataFrame [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r].
        """
        p = self.plant
        state = np.zeros(12) if x0 is None else x0.copy()
        G = 9.81
        I_vec = np.array([p.inertia.Ixx, p.inertia.Iyy, p.inertia.Izz])
        drag = np.array([p.drag.kD_xy, p.drag.kD_xy, p.drag.kD_z])
        N = min(len(torque_Nm), len(thrust_N))

        records = []
        for i in range(N):
            tau = torque_Nm.iloc[i][["tau_x", "tau_y", "tau_z"]].to_numpy(dtype=float)
            F = float(thrust_N.iloc[i]["F"])            # N, positive upward

            pos, vel, euler, omega = state[0:3], state[3:6], state[6:9], state[9:12]
            R = _euler_to_R(*euler)
            thrust_body = np.array([0.0, 0.0, -F])      # upward in body = -z_body
            a_world = R @ thrust_body / p.mass_kg + np.array([0.0, 0.0, G])
            a_drag = -drag * vel / p.mass_kg

            alpha = (tau - np.cross(omega, I_vec * omega)) / I_vec
            alpha *= dt / (p.tau_motor_s + dt)           # first-order motor lag

            state[0:3] = pos + vel * dt
            state[3:6] = vel + (a_world + a_drag) * dt
            state[6:9] = euler + omega * dt
            state[9:12] = omega + alpha * dt
            records.append(state.copy())

        cols = ["x", "y", "z", "vx", "vy", "vz", "roll", "pitch", "yaw", "p", "q", "r"]
        return pd.DataFrame(records, columns=cols)  # type: ignore[arg-type]


    def run_translational(
        self,
        thrust_N: pd.DataFrame,
        euler_real: pd.DataFrame,
        dt: float = 0.004,
        x0: Optional[np.ndarray] = None,
    ) -> pd.DataFrame:
        """
        Translational-only simulation — uses the actual recorded attitude.

        Avoids the fundamental problem of open-loop 6DOF simulation (torque bias
        accumulation): attitude is taken directly from the flight log, not integrated.
        Only position and velocity are propagated forward.

        thrust_N:    DataFrame with column [F] in Newtons, positive = upward.
                     Derived from vehicle_thrust_setpoint: F = kT * 4 * xyz[2]**2
        euler_real:  DataFrame with columns [roll, pitch, yaw] in radians,
                     interpolated from vehicle_attitude quaternion onto t_sim.
        x0:          initial state [x, y, z, vx, vy, vz, ...].  Only [0:6] used.

        Returns DataFrame [x, y, z, vx, vy, vz].
        """
        p = self.plant
        state = np.zeros(6) if x0 is None else x0[:6].copy()
        G = 9.81
        drag_xy = p.drag.kD_xy
        drag_z  = p.drag.kD_z
        N = min(len(thrust_N), len(euler_real))

        z_ground = float(state[2]) + 0.05  # NED: positive z = down; clamp prevents falling below takeoff

        records = []
        for i in range(N):
            F    = float(thrust_N.iloc[i]["F"])
            roll = float(euler_real.iloc[i]["roll"])
            pitch = float(euler_real.iloc[i]["pitch"])
            yaw  = float(euler_real.iloc[i]["yaw"])

            pos, vel = state[0:3], state[3:6]
            R = _euler_to_R(roll, pitch, yaw)
            thrust_body = np.array([0.0, 0.0, -F])
            a_world = R @ thrust_body / p.mass_kg + np.array([0.0, 0.0, G])
            drag_vec = np.array([drag_xy, drag_xy, drag_z])
            a_drag = -drag_vec * vel / p.mass_kg

            state[0:3] = pos + vel * dt
            state[3:6] = vel + (a_world + a_drag) * dt

            # Ground clamp: drone cannot fall below takeoff elevation
            if state[2] > z_ground:
                state[2] = z_ground
                state[5] = 0.0   # zero vertical velocity on ground contact

            records.append(state.copy())

        return pd.DataFrame(records, columns=["x", "y", "z", "vx", "vy", "vz"])  # type: ignore[arg-type]


def compute_rmse(simulated: pd.DataFrame, reference: pd.DataFrame,
                  cols: Optional[list] = None) -> dict:
    """RMSE per column between two DataFrames of the same length."""
    effective_cols: list = simulated.columns.tolist() if cols is None else cols
    n = min(len(simulated), len(reference))
    return {
        col: float(np.sqrt(np.mean(
            (simulated[col].to_numpy()[:n] - reference[col].to_numpy()[:n]) ** 2
        )))
        for col in effective_cols
    }
