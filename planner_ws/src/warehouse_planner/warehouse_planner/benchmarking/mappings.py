from __future__ import annotations

STATIONS = ["A", "B", "C", "D", "E", "F", "G"]

LOCATION_TO_INT = {"ON_TRANSIT": 0, **{s: i + 1 for i, s in enumerate(STATIONS)}}
INT_TO_LOCATION = {v: k for k, v in LOCATION_TO_INT.items()}

PKG_LOCATION_TO_INT = {**{s: i for i, s in enumerate(STATIONS)}, "ROBOT": 7}
INT_TO_PKG_LOCATION = {v: k for k, v in PKG_LOCATION_TO_INT.items()}

SHIP_TYPE_TO_INT = {"standard": 0, "express": 1}
INT_TO_SHIP_TYPE = {v: k for k, v in SHIP_TYPE_TO_INT.items()}

LIFECYCLE_TO_INT = {
    "SPAWNED": 0,
    "READY": 1,
    "WAITING": 2,
    "PROCESSING": 3,
    "FINISHED": 4,
    "FAILED": 5,
}
INT_TO_LIFECYCLE = {v: k for k, v in LIFECYCLE_TO_INT.items()}

NEXT_LOC_TO_INT = {
    "A": 0,
    "B": 1,
    "C": 2,
    "D": 3,
    "E": 4,
    "F": 5,
    "G": 6,
    "FINISH": 7,
}
INT_TO_NEXT_LOC = {v: k for k, v in NEXT_LOC_TO_INT.items()}
