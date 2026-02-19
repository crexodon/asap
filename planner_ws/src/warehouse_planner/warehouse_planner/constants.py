from __future__ import annotations

STATIONS = ["A", "B", "C", "D", "E", "F", "G"]
ROBOT_LOCATIONS = ["ON_TRANSIT"] + STATIONS

ACTION_TYPES = [
    "WAIT",     # 0
    "CHARGE",   # 1
    "MOVE_TO",  # 2
    "PICK",     # 3
    "DROP",     # 4
    "PICK_A",   # 5
]

# Discrete(6*20) flattened action encoding
# idx = action_type * 20 + param
FLAT_ACTIONS_N = len(ACTION_TYPES) * 20

# Lifecycle / shipping types
LIFECYCLE_STATES = ["SPAWNED", "READY", "WAITING", "PROCESSING", "FINISHED", "FAILED"]
SHIPPING_TYPES = ["standard", "express"]

# Package locations
PACKAGE_LOCATIONS = STATIONS + ["ROBOT"]

# Next-locations
NEXT_LOCATIONS = STATIONS + ["FINISH"]
