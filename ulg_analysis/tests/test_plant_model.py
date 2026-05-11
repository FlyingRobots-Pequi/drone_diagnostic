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
        assert loaded == model
    finally:
        os.unlink(path)


def test_roundtrip_json_none_fit_rmse():
    model = PlantModel(
        mass_kg=1.5,
        kT=1e-6,
        tau_motor_s=0.02,
        inertia=Inertia(Ixx=0.01, Iyy=0.01, Izz=0.02),
        drag=Drag(kD_xy=0.1, kD_z=0.2),
        arm_length_m=0.15,
    )
    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
        path = f.name
    try:
        model.save(path)
        assert PlantModel.load(path) == model
    finally:
        os.unlink(path)
