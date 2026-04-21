from dataclasses import dataclass


@dataclass
class HazardObservation:
    hazard_id: int
    bearing_deg: float
    range_m: float
    x_map: float
    y_map: float


@dataclass
class HazardEntry:
    hazard_id: int
    x_map: float
    y_map: float
    count: int