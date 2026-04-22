import math
from search_and_nav.types import HazardEntry


class MarkerDB:
    def __init__(self, duplicate_distance_threshold=0.45, min_confirmations=3):
        self.duplicate_distance_threshold = duplicate_distance_threshold
        self.min_confirmations = min_confirmations
        self.entries = []

    def add_observation(self, hazard_id, x_map, y_map):
        for entry in self.entries:
            if entry.hazard_id != hazard_id:
                continue
            dist = math.hypot(entry.x_map - x_map, entry.y_map - y_map)
            if dist <= self.duplicate_distance_threshold:
                entry.count += 1
                alpha = 1.0 / entry.count
                entry.x_map = (1 - alpha) * entry.x_map + alpha * x_map
                entry.y_map = (1 - alpha) * entry.y_map + alpha * y_map
                return entry, False

        new_entry = HazardEntry(
            hazard_id=hazard_id,
            x_map=x_map,
            y_map=y_map,
            count=1
        )
        self.entries.append(new_entry)
        return new_entry, True

    def confirmed_entries(self):
        return [e for e in self.entries if e.count >= self.min_confirmations]