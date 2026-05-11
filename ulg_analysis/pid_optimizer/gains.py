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
    (0.01, 0.5),    # rollrate_P
    (0.0,  0.5),    # rollrate_I
    (0.0,  0.012),  # rollrate_D
    (0.1,  3.0),    # rollrate_K
    (0.01, 0.5),    # pitchrate_P
    (0.0,  0.5),    # pitchrate_I
    (0.0,  0.012),  # pitchrate_D
    (0.1,  3.0),    # pitchrate_K
    (0.01, 0.5),    # yawrate_P
    (0.0,  0.5),    # yawrate_I
    (0.0,  0.010),  # yawrate_D
    (0.1,  3.0),    # yawrate_K
    (1.0,  12.0),   # roll_P
    (1.0,  12.0),   # pitch_P
    (0.5,  5.0),    # yaw_P
    (0.1,  5.0),    # xy_vel_P
    (0.0,  5.0),    # xy_vel_I
    (0.0,  2.0),    # xy_vel_D
    (0.1,  8.0),    # z_vel_P
    (0.0,  5.0),    # z_vel_I
    (0.0,  2.0),    # z_vel_D
    (0.1,  2.0),    # xy_pos_P
    (0.1,  2.0),    # z_pos_P
]

# Hermit reference gains (from log_12_2024-8-30-16-21-54.ulg)
HERMIT_REFERENCE_GAINS = Gains()
