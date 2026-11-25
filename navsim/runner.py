from pathlib import Path

import yaml

from navsim.geom import Rect
from navsim.sim import RunResult, run_world
from navsim.world import World


def load_scenario(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"场景格式错误: {path}")
    return data


def run_scenario(scenario: dict) -> RunResult:
    bounds = scenario["bounds"]
    obstacles = [Rect(**item) for item in scenario.get("obstacles", [])]
    world = World(
        width=float(bounds["width"]),
        height=float(bounds["height"]),
        obstacles=obstacles,
        robot_radius=float(scenario["robot_radius_m"]),
    )
    start = scenario["start"]
    goal = scenario["goal"]
    return run_world(
        world,
        (float(start["x"]), float(start["y"]), float(start["yaw"])),
        (float(goal["x"]), float(goal["y"])),
        float(scenario["goal_tolerance_m"]),
        float(scenario["timeout_sec"]),
    )


def format_result(scenario: dict, result: RunResult) -> str:
    odom_hz = result.hz("/odom")
    scan_hz = result.hz("/scan")
    return (
        f"{scenario['name']}: {result.status} "
        f"pose=({result.x:.2f},{result.y:.2f}) "
        f"goal_distance={result.goal_distance:.2f}m "
        f"collided={result.collided} "
        f"/odom={odom_hz:.1f}Hz /scan={scan_hz:.1f}Hz"
    )
