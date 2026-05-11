from .gains import Gains

_PX4_TO_FIELD = {
    "MC_ROLLRATE_P": "rollrate_P",
    "MC_ROLLRATE_I": "rollrate_I",
    "MC_ROLLRATE_D": "rollrate_D",
    "MC_ROLLRATE_K": "rollrate_K",
    "MC_PITCHRATE_P": "pitchrate_P",
    "MC_PITCHRATE_I": "pitchrate_I",
    "MC_PITCHRATE_D": "pitchrate_D",
    "MC_PITCHRATE_K": "pitchrate_K",
    "MC_YAWRATE_P": "yawrate_P",
    "MC_YAWRATE_I": "yawrate_I",
    "MC_YAWRATE_D": "yawrate_D",
    "MC_YAWRATE_K": "yawrate_K",
    "MC_ROLL_P": "roll_P",
    "MC_PITCH_P": "pitch_P",
    "MC_YAW_P": "yaw_P",
    "MPC_XY_VEL_P_ACC": "xy_vel_P",
    "MPC_XY_VEL_I_ACC": "xy_vel_I",
    "MPC_XY_VEL_D_ACC": "xy_vel_D",
    "MPC_Z_VEL_P_ACC": "z_vel_P",
    "MPC_Z_VEL_I_ACC": "z_vel_I",
    "MPC_Z_VEL_D_ACC": "z_vel_D",
    "MPC_XY_P": "xy_pos_P",
    "MPC_Z_P": "z_pos_P",
}


def save_params(gains: Gains, path: str) -> None:
    px4_params = gains.to_px4_params()

    with open(path, "w") as f:
        f.write("# Exported by pid_optimizer\n")
        for key in sorted(px4_params.keys()):
            f.write(f"{key}\t{px4_params[key]:.6g}\n")


def load_params(path: str) -> Gains:
    gains_dict = {}
    with open(path) as f:
        for line in f:
            line = line.rstrip("\n")
            if not line or line.startswith("#"):
                continue
            parts = line.split("\t")
            if len(parts) != 2:
                continue
            px4_key, value_str = parts
            if px4_key not in _PX4_TO_FIELD:
                continue
            gains_dict[_PX4_TO_FIELD[px4_key]] = float(value_str)
    return Gains(**gains_dict)
