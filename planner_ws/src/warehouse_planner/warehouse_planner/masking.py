from __future__ import annotations

import numpy as np

from .constants import ACTION_TYPES, FLAT_ACTIONS_N, STATIONS
from .encoding import PACKAGE_LOCATION_TO_IDX, ROBOT_LOCATION_TO_IDX

DEBUG_MASK = False


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
    episode_step = int(obs.get("episode_step", np.array([10], dtype=np.int64))[0])

    pkg_loc = obs["package_location"]  # (20,)
    pkg_next = obs["package_next_station"]
    pkg_avail = obs["package_availability"]

    on_transit_idx = ROBOT_LOCATION_TO_IDX["ON_TRANSIT"] 
    station_a_idx = ROBOT_LOCATION_TO_IDX["A"]
    station_f_idx = ROBOT_LOCATION_TO_IDX["F"]
    station_e_idx = ROBOT_LOCATION_TO_IDX["E"]

    # --- Forced prefix actions per episode (exactly one action allowed) ---
    # 0) MOVE_TO A (2*20+0=40)
    # 1) PICK_A 0 (5*20+0=100)
    # 2) MOVE_TO B (2*20+1=41)
    # 3) DROP 0 (4*20+0=80)
    forced = {0: 40, 1: 100, 2: 41, 3: 80}
    if episode_step in forced:
        mask[forced[episode_step]] = True
        return mask


    # If robot is in transit, no commands should be set (safety):
    # only WAIT remains available.
    if robot_loc_idx == on_transit_idx:
        return mask
    
    # WAIT only not carrying a package
    if carrying == -1:
        mask[type_param_to_flat(0, 0)] = True
    
    # Current station param for MOVE_TO (0..6 for A..G).
    # robot_location is encoded as: 0=ON_TRANSIT, 1..7=A..G
    # MOVE_TO params are encoded as: 0..6=A..G
    current_station_param: int | None = None
    if 1 <= robot_loc_idx <= len(STATIONS):
        current_station_param = robot_loc_idx - 1

    # CHARGE only when at F and battery < 95 (avoid double-charging at full)
    if robot_loc_idx == station_f_idx and battery < 95.0:
         for p in range(20):
             mask[type_param_to_flat(1, 0)] = True

    # MOVE_TO (only when not ON_TRANSIT - already returned above)
    station_param_f = STATIONS.index("F")  # 5
    allow_f = (battery < 50.0)

    if carrying == -1:
        # MOVE_TO A only if empty (we are empty here)
        mask[type_param_to_flat(2, STATIONS.index("A"))] = True

        # MOVE_TO B/C/G only if there exists at least one package currently located there
        for st in ["B", "C", "G"]:
            st_idx = PACKAGE_LOCATION_TO_IDX[st]
            if np.any(pkg_loc == st_idx):
                mask[type_param_to_flat(2, STATIONS.index(st))] = True

        # MOVE_TO D only if there exists at least one package at D that is available
        d_idx = PACKAGE_LOCATION_TO_IDX["D"]
        if np.any((pkg_loc == d_idx) & (pkg_avail == 1)):
            mask[type_param_to_flat(2, STATIONS.index("D"))] = True

        # MOVE_TO E only if carrying != -1 (never here), so do nothing

        # MOVE_TO F only if battery < 50
        if allow_f:
            mask[type_param_to_flat(2, station_param_f)] = True
        

    else:
        # carrying: allow to go to next_station of that package (if A..G), plus F (if battery < 50)
        next_idx = int(pkg_next[carrying])  # 0..6 stations, 7 == FINISH
        if 0 <= next_idx <= 6:
            mask[type_param_to_flat(2, next_idx)] = True
        if allow_f:
            mask[type_param_to_flat(2, station_param_f)] = True

    
    # Forbid MOVE_TO to the station where the robot already is (no-op move).
    # Apply after MOVE_TO candidates have been enabled, so it works for both
    # "carrying" and "not carrying" branches.
    if current_station_param is not None and 0 <= current_station_param <= 6:
        mask[type_param_to_flat(2, current_station_param)] = False
    # PICK (explicitly only at B, C, D, G)
    # - robot_loc in {B,C,D,G}
    # - carrying == -1
    # - package at same station AND availability True
    if carrying == -1:
        allowed_pick_stations = {"B", "C", "D", "G"}
        # resolve current station name
        station_name = None
        for k, v in ROBOT_LOCATION_TO_IDX.items():
            if v == robot_loc_idx:
                station_name = k
                break
        if station_name in allowed_pick_stations:
            station_pkg_idx = PACKAGE_LOCATION_TO_IDX[station_name]
            for i in range(20):
                if int(pkg_loc[i]) == station_pkg_idx and int(pkg_avail[i]) == 1:
                    mask[type_param_to_flat(3, i)] = True

    # DROP
    # - robot_loc in {B,C,D,E,G}
    # - carrying != -1
    # - only DROP carrying
    # - only if next_station of carried pkg == current station (and next != FINISH)
    if carrying != -1 and robot_loc_idx not in (station_a_idx, station_f_idx):

         # robot station name
        station_name = None
        for k, v in ROBOT_LOCATION_TO_IDX.items():
            if v == robot_loc_idx:
                station_name = k
                break
        if station_name is not None and station_name in STATIONS:
            station_pkg_idx = PACKAGE_LOCATION_TO_IDX[station_name]
            nxt = int(pkg_next[carrying])
            if 0 <= nxt <= 6 and nxt == station_pkg_idx:
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

    if DEBUG_MASK:
            allowed = np.where(mask)[0]

            def flat_to_str(a):
                atype = int(a) // 20
                param = int(a) % 20
                # Korrektur: Zugriff auf Liste statt .get()
                if 0 <= atype < len(ACTION_TYPES):
                    name = ACTION_TYPES[atype]
                else:
                    name = f"UNK({atype})"
                return f"{name}({param})"

            # Resolve robot location name
            loc_name = "UNKNOWN"
            for k, v in ROBOT_LOCATION_TO_IDX.items():
                if v == robot_loc_idx:
                    loc_name = k
                    break

            print("\n[Mask Debug]")
            print(f"  episode_step: {episode_step}")
            print(f"  robot_location: {loc_name}")
            print(f"  carrying: {carrying}")
            print(f"  battery: {battery:.1f}")
            print(f"  allowed_actions ({len(allowed)}):")
            for a in allowed:
                print(f"    - {flat_to_str(a)}")

    return mask