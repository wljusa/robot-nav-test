from pathlib import Path

import yaml

from navsim.geom import MovingRect, Rect
from navsim.health import check_topics
from navsim.sim import RunResult, run_world
from navsim.world import World


def load_scenario(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"场景格式错误: {path}")
    return data


def run_scenario(scenario: dict) -> RunResult:
    bounds = scenario["bounds"]
    world = World(
        width=float(bounds["width"]),
        height=float(bounds["height"]),
        obstacles=[Rect(**item) for item in scenario.get("obstacles", [])],
        robot_radius=float(scenario["robot_radius_m"]),
        motions=[MovingRect(**item) for item in scenario.get("motions", [])],
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


def expectation_failures(scenario: dict, result: RunResult) -> list[str]:
    expect = scenario.get("expect", {})
    failures: list[str] = []
    if "result" in expect and result.status != expect["result"]:
        failures.append(f"状态 {result.status}，期望 {expect['result']}")
    if "collided" in expect and result.collided is not expect["collided"]:
        failures.append(f"碰撞 {result.collided}，期望 {expect['collided']}")
    if "odom_hz_min" in expect:
        if result.hz("/odom") < expect["odom_hz_min"]:
            failures.append(f"/odom 频率 {result.hz('/odom'):.1f}Hz 低于 {expect['odom_hz_min']}")
        if not result.monotonic("/odom"):
            failures.append("/odom 时间戳回退")
    if "scan_hz_min" in expect:
        if result.hz("/scan") < expect["scan_hz_min"]:
            failures.append(f"/scan 频率 {result.hz('/scan'):.1f}Hz 低于 {expect['scan_hz_min']}")
        if not result.monotonic("/scan"):
            failures.append("/scan 时间戳回退")
    if expect.get("odom_monotonic") and not result.monotonic("/odom"):
        failures.append("/odom 时间戳回退")
    if "imu_hz_min" in expect and result.hz("/imu") < expect["imu_hz_min"]:
        failures.append(f"/imu 频率 {result.hz('/imu'):.1f}Hz 低于 {expect['imu_hz_min']}")
    if "min_clearance_m" in expect and result.min_clearance < float(expect["min_clearance_m"]):
        failures.append(f"最小间隙 {result.min_clearance:.3f}m 低于 {expect['min_clearance_m']}")
    if "max_path_length_m" in expect and result.path_length > float(expect["max_path_length_m"]):
        failures.append(f"路径长度 {result.path_length:.2f}m 超过 {expect['max_path_length_m']}")
    if "max_linear" in expect and result.max_linear() > float(expect["max_linear"]):
        failures.append(f"线速度 {result.max_linear():.2f} 超过 {expect['max_linear']}")
    if "max_abs_angular" in expect and result.max_abs_angular() > float(expect["max_abs_angular"]):
        failures.append(f"角速度 {result.max_abs_angular():.2f} 超过 {expect['max_abs_angular']}")
    if "scan_beams" in expect:
        scans = result.topics.get("/scan", [])
        if not scans or any(len(msg.data.get("ranges", [])) != expect["scan_beams"] for msg in scans):
            failures.append(f"激光束数不是 {expect['scan_beams']}")
    if expect.get("health"):
        for finding in check_topics(result.topics):
            failures.append(str(finding))
    return failures


def format_result(scenario: dict, result: RunResult) -> str:
    clearance = "n/a" if result.min_clearance == math.inf else f"{result.min_clearance:.2f}m"
    return (
        f"{scenario['name']}: {result.status} "
        f"pose=({result.x:.2f},{result.y:.2f}) "
        f"goal_distance={result.goal_distance:.2f}m "
        f"path={result.path_length:.2f}m "
        f"clearance={clearance} "
        f"collided={result.collided} "
        f"/odom={result.hz('/odom'):.1f}Hz /imu={result.hz('/imu'):.1f}Hz /scan={result.hz('/scan'):.1f}Hz"
    )
