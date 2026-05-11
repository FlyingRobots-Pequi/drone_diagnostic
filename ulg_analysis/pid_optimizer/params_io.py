"""I/O for PX4 .params files (QGroundControl text format)."""

from .gains import Gains

# Build reverse mapping: PX4 key -> field name
# Based on Gains.to_px4_params() method in gains.py
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
    """Save Gains to a PX4 .params file (QGroundControl text format).

    Format:
        # Exported by pid_optimizer
        KEY1\tVALUE1
        KEY2\tVALUE2
        ...

    Args:
        gains: Gains object to save
        path: Output file path
    """
    px4_params = gains.to_px4_params()

    with open(path, "w") as f:
        f.write("# Exported by pid_optimizer\n")
        for key in sorted(px4_params.keys()):
            value = px4_params[key]
            # Format with .6g: compact, no trailing zeros
            f.write(f"{key}\t{value:.6g}\n")


def load_params(path: str) -> Gains:
    """Load Gains from a PX4 .params file.

    Starts with Gains() defaults and updates only fields present in the file.
    Lines starting with '#' are comments and skipped.
    Unknown keys are silently ignored.

    Args:
        path: Input file path

    Returns:
        Gains object with values from file (or defaults for missing keys)
    """
    gains_dict = {}

    with open(path, "r") as f:
        for line in f:
            line = line.rstrip("\n")
            # Skip empty lines and comments
            if not line or line.startswith("#"):
                continue

            # Parse as tab-separated key-value
            parts = line.split("\t")
            if len(parts) != 2:
                continue

            px4_key, value_str = parts

            # Map PX4 key to field name
            if px4_key not in _PX4_TO_FIELD:
                # Unknown key, silently ignore
                continue

            field_name = _PX4_TO_FIELD[px4_key]
            try:
                gains_dict[field_name] = float(value_str)
            except ValueError:
                # Malformed value, skip this line
                continue

    # Create Gains with defaults, then update with loaded values
    return Gains(**gains_dict)
