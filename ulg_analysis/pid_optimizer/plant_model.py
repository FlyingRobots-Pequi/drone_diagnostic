from dataclasses import dataclass, asdict
import json
from typing import Optional


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
    fit_rmse: Optional[dict] = None

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
