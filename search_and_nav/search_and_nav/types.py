from dataclasses import dataclass

@dataclass
class HazardEntry:
    hazard_id: int
    x_map: float
    y_map: float
    count: int = 1