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
