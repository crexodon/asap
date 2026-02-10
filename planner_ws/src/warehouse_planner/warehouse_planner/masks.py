from __future__ import annotations
from typing import Dict, List, Tuple

# Action space definition (discrete)
# You kannst das später erweitern/ändern.
# Index -> (action_name, param)
ACTIONS: List[Tuple[str, Dict[str, str]]] = [
    ("WAIT", {}),
    ("CHARGE", {}),
    ("PICK", {}),   # pick some available package at current station (planner chooses via param in command)
    ("DROP", {}),   # drop carried package at current station (only if it's next_station)
    ("MOVE_TO", {"station": "A"}),
    ("MOVE_TO", {"station": "B"}),
    ("MOVE_TO", {"station": "C"}),
    ("MOVE_TO", {"station": "D"}),
    ("MOVE_TO", {"station": "E"}),
    ("MOVE_TO", {"station": "F"}),
    ("MOVE_TO", {"station": "G"}),
    ("MOVE_TO", {"station": "S"}),
]

STATIONS = {"A", "B", "C", "D", "E", "F", "G", "S"}


def compute_action_mask(world: Dict) -> List[bool]:
    """
    Returns a boolean mask aligned with ACTIONS[] (True = allowed).
    Only physical/logical validity, per your rule.
    """
    mask = [True] * len(ACTIONS)

    loc = str(world.get("robot_location", "S"))
    carrying = int(world.get("robot_carrying_idx", -1))
    battery = float(world.get("battery", 0.0))
    packages = list(world.get("packages", []))

    # CHARGE only at F
    idx_charge = 1
    mask[idx_charge] = (loc == "F")

    # PICK only if not carrying and at least one pickable package at this station
    idx_pick = 2
    if carrying != -1:
        mask[idx_pick] = False
    else:
        pickable = False
        for p in packages:
            if str(p.get("location")) == loc and bool(p.get("availability", False)):
                # also ensure it is in a state that can be picked (optional)
                pickable = True
                break
        mask[idx_pick] = pickable

    # DROP only if carrying and current station == next_station of carried package
    idx_drop = 3
    if carrying == -1:
        mask[idx_drop] = False
    else:
        if carrying < 0 or carrying >= len(packages):
            mask[idx_drop] = False
        else:
            next_station = str(packages[carrying].get("next_station", ""))
            mask[idx_drop] = (loc == next_station)

    # MOVE_TO validity:
    # - always allowed in principle
    # - optionally forbid MOVE_TO(current_loc)
    for i, (name, param) in enumerate(ACTIONS):
        if name == "MOVE_TO":
            st = param.get("station", "")
            if st not in STATIONS:
                mask[i] = False
            elif st == loc:
                mask[i] = False  # no-op move
            else:
                mask[i] = True

    # WAIT always allowed (index 0)
    mask[0] = True

    # Optional: hard battery safety.
    # If battery is already <= 0, no action is valid; but your env should reset before this.
    if battery <= 0.0:
        mask = [False] * len(ACTIONS)

    return mask


def action_index_to_command(action_index: int, world: Dict) -> Dict:
    """
    Converts discrete action index into a PlannerCommand JSON dict.
    For PICK, we also choose WHICH package to pick (greedy: first available at loc).
    You can replace this with your own selection logic or extend action space.
    """
    action_name, param_template = ACTIONS[action_index]
    cmd = {"action": action_name, "params": {}}

    if action_name == "MOVE_TO":
        cmd["params"] = {"station": param_template["station"]}
        return cmd

    if action_name == "CHARGE":
        cmd["params"] = {}
        return cmd

    if action_name == "WAIT":
        cmd["params"] = {}
        return cmd

    if action_name == "DROP":
        cmd["params"] = {}
        return cmd

    if action_name == "PICK":
        loc = str(world.get("robot_location", "S"))
        packages = list(world.get("packages", []))
        chosen = None
        for i, p in enumerate(packages):
            if str(p.get("location")) == loc and bool(p.get("availability", False)):
                chosen = i
                break
        cmd["params"] = {"package_idx": chosen if chosen is not None else -1}
        return cmd

    # fallback
    return cmd
