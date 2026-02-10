from __future__ import annotations

import json
import time
from dataclasses import dataclass, asdict
from typing import Any, Dict, List, Optional


def now_wall() -> float:
    return time.time()


def json_dumps(obj: Any) -> str:
    return json.dumps(obj, separators=(",", ":"), ensure_ascii=False)


def json_loads(s: str) -> Any:
    return json.loads(s)


@dataclass
class Event:
    """
    Generic trigger event coming from ROS topics (JSON-encoded).
    """
    type: str
    stamp: float  # wall time seconds (or sim time if you want)
    payload: Dict[str, Any]

    @staticmethod
    def from_json(data: Dict[str, Any]) -> "Event":
        return Event(
            type=str(data.get("type", "UNKNOWN")),
            stamp=float(data.get("stamp", now_wall())),
            payload=dict(data.get("payload", {})),
        )


@dataclass
class WorldState:
    """
    This is what the aggregator publishes to the planner (JSON-encoded).

    Keep it stable. Add fields rather than renaming them.
    """
    stamp: float
    decision_required: bool
    dt_since_last_decision: float
    last_events: List[Dict[str, Any]]

    # robot state
    robot_location: str
    robot_carrying_idx: int  # -1 if none
    battery: float

    # packages: fixed-size vector (20)
    packages: List[Dict[str, Any]]  # each: {location, availability, next_station, shipping_type, lifecycle_state}

    # bookkeeping
    jobs_done: int
    jobs_failed_to_E: int
    episode_time: float  # seconds since reset

    def to_json(self) -> str:
        return json_dumps(asdict(self))


@dataclass
class PlannerCommand:
    """
    Planner output command to your execution layer (JSON-encoded).
    """
    stamp: float
    action: str
    params: Dict[str, Any]

    def to_json(self) -> str:
        return json_dumps(asdict(self))
