from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np

from .constants import (
    LIFECYCLE_STATES,
    PACKAGE_LOCATIONS,
    ROBOT_LOCATIONS,
    SHIPPING_TYPES,
    STATIONS,
)


def _make_map(items: List[str]) -> Dict[str, int]:
    return {name: i for i, name in enumerate(items)}


ROBOT_LOCATION_TO_IDX = _make_map(ROBOT_LOCATIONS)
PACKAGE_LOCATION_TO_IDX = _make_map(PACKAGE_LOCATIONS)
STATION_TO_IDX = _make_map(STATIONS)
SHIPPING_TO_IDX = _make_map(SHIPPING_TYPES)
LIFECYCLE_TO_IDX = _make_map(LIFECYCLE_STATES)


@dataclass
class EncodedState:
    """Observation in a SB3-friendly numeric form."""

    battery_status: float
    robot_location: int
    robot_carrying_idx: int
    delta_time: float

    package_location: np.ndarray  # (20,) int
    package_next_station: np.ndarray  # (20,) int
    package_shipping_type: np.ndarray  # (20,) int
    package_lifecycle_state: np.ndarray  # (20,) int
    package_availability: np.ndarray  # (20,) int (0/1)

    def as_dict(self) -> Dict[str, np.ndarray]:
        return {
            "battery_status": np.array([self.battery_status], dtype=np.float32),
            "robot_location": np.array([self.robot_location], dtype=np.int64),
            "robot_carrying_idx": np.array([self.robot_carrying_idx], dtype=np.int64),
            "delta_time": np.array([self.delta_time], dtype=np.float32),
            "package_location": self.package_location.astype(np.int64),
            "package_next_station": self.package_next_station.astype(np.int64),
            "package_shipping_type": self.package_shipping_type.astype(np.int64),
            "package_lifecycle_state": self.package_lifecycle_state.astype(np.int64),
            "package_availability": self.package_availability.astype(np.int64),
        }


def safe_idx(mapping: Dict[str, int], key: str, default: int = 0) -> int:
    return mapping.get(key, default)


def carrying_from_packages(package_locations: np.ndarray) -> int:
    """Derive carrying idx from package current_location == ROBOT.

    If multiple are ROBOT, first one wins (should not happen).
    Determines which package is currently carried by the robot
    """
    robot_idx = PACKAGE_LOCATION_TO_IDX.get("ROBOT", -1)
    if robot_idx < 0:
        return -1
    matches = np.where(package_locations == robot_idx)[0]
    return int(matches[0]) if len(matches) > 0 else -1
