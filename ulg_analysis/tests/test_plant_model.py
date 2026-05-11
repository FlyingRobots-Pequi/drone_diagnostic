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
