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
