from __future__ import annotations

import math
import pathlib

import numpy as np
import pytest
from arena_planners.sdk import load_manifest

import planner

_MANIFEST = load_manifest(pathlib.Path(planner.__file__).parent / "planner.yaml")


@pytest.fixture(autouse=True)
def _manifest_config() -> None:
    planner.on_init(_MANIFEST["config"])


def _features(robot: tuple[float, float, float], goal: tuple[float, float, float]) -> dict:
    return {"robot_pose": np.array(robot), "goal_pose": np.array(goal)}


def test_drives_straight_at_a_goal_ahead() -> None:
    v, omega = planner.step(_features((0.0, 0.0, 0.0), (5.0, 0.0, 0.0)))
    assert v == pytest.approx(_MANIFEST["config"]["max_linear"])
    assert omega == pytest.approx(0.0)


def test_turns_in_place_toward_a_goal_behind() -> None:
    v, omega = planner.step(_features((0.0, 0.0, 0.0), (-5.0, 0.1, 0.0)))
    assert v == 0.0
    assert omega == pytest.approx(_MANIFEST["config"]["max_angular"])


def test_turns_toward_a_goal_to_the_right() -> None:
    _, omega = planner.step(_features((0.0, 0.0, 0.0), (5.0, -1.0, 0.0)))
    assert omega < 0.0


def test_stops_inside_the_goal_tolerance() -> None:
    assert planner.step(_features((1.0, 1.0, 0.0), (1.05, 1.0, 0.0))) == [0.0, 0.0]


def test_holds_still_without_a_goal() -> None:
    assert planner.step({"robot_pose": np.zeros(3)}) == [0.0, 0.0]


def test_reaches_the_goal_on_a_unicycle() -> None:
    pose = np.array([0.0, 0.0, math.pi / 2])
    goal = (3.0, -2.0, 0.0)
    dt = 1.0 / 10
    for _ in range(400):
        v, omega = planner.step({"robot_pose": pose, "goal_pose": np.array(goal)})
        if v == 0.0 and omega == 0.0:
            break
        pose = pose + dt * np.array([v * math.cos(pose[2]), v * math.sin(pose[2]), omega])
    assert math.hypot(goal[0] - pose[0], goal[1] - pose[1]) < _MANIFEST["config"]["goal_tolerance"]
