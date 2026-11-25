from pathlib import Path

import pytest

from navsim.runner import load_scenario, run_scenario

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = sorted((ROOT / "scenarios").glob("*.yaml"))


@pytest.mark.parametrize("path", SCENARIOS, ids=lambda item: item.stem)
def test_scenario(path: Path):
    scenario = load_scenario(path)
    result = run_scenario(scenario)
    expect = scenario["expect"]
    assert result.status == expect["result"]
    if "collided" in expect:
        assert result.collided is expect["collided"]
    if "odom_hz_min" in expect:
        assert result.hz("/odom") >= expect["odom_hz_min"]
        assert result.monotonic("/odom")
    if "scan_hz_min" in expect:
        assert result.hz("/scan") >= expect["scan_hz_min"]
        assert result.monotonic("/scan")
    if expect.get("odom_monotonic"):
        assert result.monotonic("/odom")
