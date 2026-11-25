from copy import deepcopy
from pathlib import Path

from navsim.health import check_topics
from navsim.runner import load_scenario, run_scenario
from navsim.sim import TopicMsg

ROOT = Path(__file__).resolve().parents[1]


def _reach():
    scenario = load_scenario(ROOT / "scenarios" / "reach_goal.yaml")
    return run_scenario(scenario)


def test_healthy_run_passes():
    assert check_topics(_reach().topics) == []


def test_scan_drop_is_a_gap():
    topics = deepcopy(_reach().topics)
    scan = topics["/scan"]
    topics["/scan"] = scan[:3] + scan[12:]
    codes = {(item.code, item.topic) for item in check_topics(topics)}
    assert ("gap", "/scan") in codes


def test_imu_rewind_is_monotonic():
    topics = deepcopy(_reach().topics)
    imu = topics["/imu"]
    topics["/imu"] = [
        *imu[:40],
        TopicMsg(imu[37].stamp_ns, imu[40].data),
        *imu[41:],
    ]
    codes = {(item.code, item.topic) for item in check_topics(topics)}
    assert ("monotonic", "/imu") in codes


def test_short_scan_beams_fail():
    topics = deepcopy(_reach().topics)
    first = topics["/scan"][0]
    topics["/scan"][0] = TopicMsg(first.stamp_ns, {**first.data, "ranges": first.data["ranges"][:4]})
    codes = {(item.code, item.topic) for item in check_topics(topics)}
    assert ("beams", "/scan") in codes
