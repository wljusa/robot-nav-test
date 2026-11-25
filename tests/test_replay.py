from pathlib import Path

from navsim.health import check_topics
from navsim.logio import read_log, write_log
from navsim.runner import load_scenario, run_scenario

ROOT = Path(__file__).resolve().parents[1]


def test_replay_keeps_topics_and_passes_health(tmp_path):
    scenario = load_scenario(ROOT / "scenarios" / "detour.yaml")
    result = run_scenario(scenario)
    log_path = tmp_path / "detour.jsonl"
    write_log(log_path, result.topics)
    restored = read_log(log_path)
    assert set(restored) == set(result.topics)
    assert len(restored["/scan"]) == len(result.topics["/scan"])
    assert restored["/odom"][-1].data == result.topics["/odom"][-1].data
    assert check_topics(restored) == []
