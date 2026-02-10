from __future__ import annotations

import numpy as np

from .constants import ACTION_TYPES, FLAT_ACTIONS_N, STATIONS
from .encoding import PACKAGE_LOCATION_TO_IDX, ROBOT_LOCATION_TO_IDX


def flat_to_type_param(action: int) -> tuple[int, int]:
    atype = int(action) // 20
    param = int(action) % 20
    return atype, param


def type_param_to_flat(atype: int, param: int) -> int:
    return int(atype) * 20 + int(param)


def station_param_to_station(param: int) -> str:
    # 0..6 -> A..G
    idx = int(param)
    idx = max(0, min(idx, 6))
    return STATIONS[idx]


def compute_action_mask(obs: dict) -> np.ndarray:
    """Compute boolean mask for flattened Discrete(120) action space.

    The obs is the numeric dict produced by EncodedState.as_dict().
    """
    mask = np.zeros((FLAT_ACTIONS_N,), dtype=bool)

    # Extract basics
    robot_loc_idx = int(obs["robot_location"][0])
    battery = float(obs["battery_status"][0])
    carrying = int(obs["robot_carrying_idx"][0])

    pkg_loc = obs["package_location"]  # (20,)
    pkg_next = obs["package_next_station"]
    pkg_avail = obs["package_availability"]

    on_transit_idx = ROBOT_LOCATION_TO_IDX["ON_TRANSIT"]
    station_a_idx = ROBOT_LOCATION_TO_IDX["A"]
    station_f_idx = ROBOT_LOCATION_TO_IDX["F"]
    station_e_idx = ROBOT_LOCATION_TO_IDX["E"]

    # WAIT always true (param ignored)
    for p in range(20):
        mask[type_param_to_flat(0, p)] = True

    # CHARGE only when at F
    if robot_loc_idx == station_f_idx:
        for p in range(20):
            mask[type_param_to_flat(1, p)] = True

    # MOVE_TO
    # If not carrying: any station A..G selectable
    if carrying == -1:
        for s_idx in range(7):
            # param 0..6 used; other params are masked
            mask[type_param_to_flat(2, s_idx)] = True
    else:
        # carrying: allow to go to next_station of that package, plus F (charging)
        next_idx = int(pkg_next[carrying])
        # next_idx is in PACKAGE_LOCATION_TO_IDX, we only allow if it's a station
        # Convert next_idx -> station param
        # We know mapping in PACKAGE_LOCATION_TO_IDX is STATIONS + [ROBOT]
        # so station indices align with STATIONS order.
        # Only allow A..G (0..6)
        if 0 <= next_idx <= 6:
            mask[type_param_to_flat(2, next_idx)] = True
        # Always allow F (index 5)
        mask[type_param_to_flat(2, STATIONS.index("F"))] = True

    # PICK
    # Conditions:
    # - robot at station (not ON_TRANSIT)
    # - robot_location not A or F
    # - robot_location not E
    # - carrying == -1
    # - package at same station AND availability True
    if robot_loc_idx != on_transit_idx and robot_loc_idx not in (station_a_idx, station_f_idx, station_e_idx) and carrying == -1:
        # station id string
        # package current_location uses PACKAGE_LOCATION_TO_IDX (A..G, ROBOT)
        # robot location uses ROBOT_LOCATION_TO_IDX (ON_TRANSIT, A..G)
        # station indices for A..G are the same offset by +1 in ROBOT_LOCATION
        station_name = None
        for k, v in ROBOT_LOCATION_TO_IDX.items():
            if v == robot_loc_idx:
                station_name = k
                break
        if station_name is not None and station_name in STATIONS:
            station_pkg_idx = PACKAGE_LOCATION_TO_IDX[station_name]
            for i in range(20):
                if int(pkg_loc[i]) == station_pkg_idx and int(pkg_avail[i]) == 1:
                    mask[type_param_to_flat(3, i)] = True

    # DROP
    # - robot_location not A or F
    # - carrying != -1
    # - param must equal carrying
    # - next_station of carried pkg == robot_location
    if carrying != -1 and robot_loc_idx not in (station_a_idx, station_f_idx):
        # robot station name
        station_name = None
        for k, v in ROBOT_LOCATION_TO_IDX.items():
            if v == robot_loc_idx:
                station_name = k
                break
        if station_name is not None and station_name in STATIONS:
            station_pkg_idx = PACKAGE_LOCATION_TO_IDX[station_name]
            if int(pkg_next[carrying]) == station_pkg_idx:
                mask[type_param_to_flat(4, carrying)] = True

    # PICK_A
    # - robot at A
    # - carrying == -1
    # - package_location == A and availability == True
    if robot_loc_idx == station_a_idx and carrying == -1:
        a_idx = PACKAGE_LOCATION_TO_IDX["A"]
        for i in range(20):
            if int(pkg_loc[i]) == a_idx and int(pkg_avail[i]) == 1:
                mask[type_param_to_flat(5, i)] = True

    return mask
