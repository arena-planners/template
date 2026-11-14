"""Turn-then-drive controller toward the goal pose, a planner of the arena_planners bridge."""

from __future__ import annotations

import math
import pathlib

import numpy as np
from arena_planners.sdk import load_manifest, main_loop

_config: dict = {}


def on_init(planner_config: dict) -> None:
    _config.clear()
    _config.update(planner_config)


def step(features: dict) -> list[float]:
    """Compute [v, omega] from robot_pose and goal_pose, both (x, y, theta) in the map frame."""
    robot_pose = features.get("robot_pose")
    goal_pose = features.get("goal_pose")
    if robot_pose is None or goal_pose is None:
        return [0.0, 0.0]
    dx = float(goal_pose[0] - robot_pose[0])
    dy = float(goal_pose[1] - robot_pose[1])
    distance = math.hypot(dx, dy)
    if distance < _config["goal_tolerance"]:
        return [0.0, 0.0]
    heading_error = (math.atan2(dy, dx) - float(robot_pose[2]) + math.pi) % (2 * math.pi) - math.pi
    omega = float(np.clip(_config["angular_gain"] * heading_error, -_config["max_angular"], _config["max_angular"]))
    if abs(heading_error) > _config["turn_in_place"]:
        return [0.0, omega]
    v = float(np.clip(_config["linear_gain"] * distance, 0.0, _config["max_linear"])) * math.cos(heading_error)
    return [v, omega]


def on_reset(episode_id: str, initial_state: dict | None) -> None:
    pass


if __name__ == "__main__":
    manifest = load_manifest(pathlib.Path(__file__).parent / "planner.yaml")
    main_loop(step, manifest=manifest, on_reset=on_reset, on_init=on_init)
