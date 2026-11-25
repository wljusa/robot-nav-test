from pathlib import Path

import pytest

from navsim.runner import expectation_failures, load_scenario, run_scenario

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = sorted((ROOT / "scenarios").glob("*.yaml"))


@pytest.mark.parametrize("path", SCENARIOS, ids=lambda item: item.stem)
def test_scenario(path: Path):
    scenario = load_scenario(path)
    result = run_scenario(scenario)
    assert expectation_failures(scenario, result) == []
