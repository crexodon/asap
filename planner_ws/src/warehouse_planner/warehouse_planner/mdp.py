from __future__ import annotations

import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from .constants import (
    ACTION_TYPES,
    FLAT_ACTIONS_N,
    LIFECYCLE_STATES,
    PACKAGE_LOCATIONS,
    ROBOT_LOCATIONS,
    SHIPPING_TYPES,
    STATIONS,
)
from .ros_interface import WarehouseROSInterface


def _index_or_default(value: str, vocab: List[str], default: int = 0) -> int:
    try:
        return vocab.index(value)
    except ValueError:
        return default


def build_state_dict(ros: WarehouseROSInterface, delta_time: float) -> Dict[str, Any]:
    """Fetch latest robot state (cached) and packages (service) and build a Dict observation."""
    robot = ros.get_robot_state()
    packages = ros.get_packages()

    # Ensure we have 20 packages; if not, still handle gracefully
    packages_sorted = sorted(packages, key=lambda p: int(p.package_idx))

    # Derive carrying from package current_location == ROBOT (SoT)
    carrying_idx = -1
    for p in packages_sorted:
        if str(p.current_location) == "ROBOT":
            carrying_idx = int(p.package_idx)
            break

    obs: Dict[str, Any] = {
        "battery_status": np.array([float(robot.battery)], dtype=np.float32),
        "robot_location": np.array([_index_or_default(robot.robot_location, ROBOT_LOCATIONS, 0)], dtype=np.int64),
        "robot_carrying_idx": np.array([int(carrying_idx)], dtype=np.int64),
        "delta_time": np.array([float(delta_time)], dtype=np.float32),
        "packages_location": np.zeros((20,), dtype=np.int64),
        "packages_next_station": np.zeros((20,), dtype=np.int64),
        "packages_shipping_type": np.zeros((20,), dtype=np.int64),
        "packages_lifecycle_state": np.zeros((20,), dtype=np.int64),
        "packages_availability": np.zeros((20,), dtype=np.int64),
    }

    for i in range(min(20, len(packages_sorted))):
        p = packages_sorted[i]
        obs["packages_location"][i] = _index_or_default(str(p.current_location), PACKAGE_LOCATIONS, 0)
        obs["packages_next_station"][i] = _index_or_default(str(p.next_location), STATIONS, 0)
        obs["packages_shipping_type"][i] = _index_or_default(str(p.shipping_type), SHIPPING_TYPES, 0)
        obs["packages_lifecycle_state"][i] = _index_or_default(str(p.lifecycle_state), LIFECYCLE_STATES, 0)
        obs["packages_availability"][i] = 1 if bool(p.availability) else 0

    return obs


def is_done_from_packages(packages: List[Any]) -> bool:
    """Episode done if all 20 packages have next_location == 'FINISH'."""
    if len(packages) < 20:
        return False
    for p in packages:
        if str(p.next_location) != "FINISH":
            return False
    return True


def decode_flat_action(a: int) -> Tuple[int, int]:
    a = int(a)
    action_type = a // 20
    param = a % 20
    return action_type, param


def action_to_cmd(action_type: int, param: int) -> str:
    t = ACTION_TYPES[int(action_type)]
    if t == "WAIT":
        return "WAIT"
    if t == "CHARGE":
        return "CHARGE"
    if t == "MOVE_TO":
        # param 0..6 => A..G
        station = STATIONS[int(param) % len(STATIONS)]
        return f"MOVE_TO {station}"
    if t == "PICK":
        return f"PICK {int(param)}"
    if t == "DROP":
        return f"DROP {int(param)}"
    if t == "PICK_A":
        return f"PICK_A {int(param)}"
    raise ValueError(f"Unknown action_type {action_type}")


def compute_action_mask(obs: Dict[str, Any]) -> np.ndarray:
    """Compute mask for flattened Discrete(120) action space."""
    mask = np.zeros((FLAT_ACTIONS_N,), dtype=np.int8)

    robot_loc_idx = int(obs["robot_location"][0])
    robot_loc = ROBOT_LOCATIONS[robot_loc_idx] if 0 <= robot_loc_idx < len(ROBOT_LOCATIONS) else "ON_TRANSIT"
    carrying_idx = int(obs["robot_carrying_idx"][0])

    pkg_loc = obs["packages_location"]
    pkg_next = obs["packages_next_station"]
    pkg_av = obs["packages_availability"]

    # Helper to set valid for an action_type over some params
    def allow(action_type: int, params: List[int]):
        for p in params:
            idx = action_type * 20 + int(p)
            if 0 <= idx < mask.size:
                mask[idx] = 1

    # WAIT always True (all params valid but ignored)
    allow(0, list(range(20)))

    # CHARGE only at F
    if robot_loc == "F":
        allow(1, list(range(20)))

    # MOVE_TO
    if carrying_idx == -1:
        # any station A..G => params 0..6 valid, rest invalid
        allow(2, list(range(len(STATIONS))))
    else:
        # allow next_station plus F always
        next_idx = int(pkg_next[carrying_idx]) if 0 <= carrying_idx < 20 else 0
        allowed = {next_idx}
        allowed.add(STATIONS.index("F"))
        allow(2, sorted(list(allowed)))

    # PICK rules
    # robot_location not ON_TRANSIT; not A or F; not E; carrying must be -1
    if robot_loc not in ("ON_TRANSIT", "A", "F", "E") and carrying_idx == -1:
        # For each package param
        for i in range(20):
            # package location must equal robot station and availability True
            loc_str = PACKAGE_LOCATIONS[int(pkg_loc[i])] if 0 <= int(pkg_loc[i]) < len(PACKAGE_LOCATIONS) else "A"
            if loc_str == robot_loc and int(pkg_av[i]) == 1:
                allow(3, [i])

    # DROP rules
    # not A or F; must be carrying; param must equal carrying_idx; next_station must equal robot_loc
    if robot_loc not in ("A", "F", "ON_TRANSIT") and carrying_idx != -1:
        if 0 <= carrying_idx < 20:
            next_station_str = STATIONS[int(pkg_next[carrying_idx])] if 0 <= int(pkg_next[carrying_idx]) < len(STATIONS) else "A"
            if next_station_str == robot_loc:
                allow(4, [carrying_idx])

    # PICK_A rules
    if robot_loc == "A" and carrying_idx == -1:
        for i in range(20):
            loc_str = PACKAGE_LOCATIONS[int(pkg_loc[i])] if 0 <= int(pkg_loc[i]) < len(PACKAGE_LOCATIONS) else "A"
            if loc_str == "A" and int(pkg_av[i]) == 1:
                allow(5, [i])

    return mask
