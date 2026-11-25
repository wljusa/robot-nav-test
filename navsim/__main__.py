import sys
from pathlib import Path

from navsim.runner import format_result, load_scenario, run_scenario


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if len(args) != 1:
        print("用法: python -m navsim scenarios/reach_goal.yaml", file=sys.stderr)
        return 2
    scenario = load_scenario(Path(args[0]))
    result = run_scenario(scenario)
    print(format_result(scenario, result))
    expect = scenario.get("expect", {}).get("result")
    return 0 if expect is None or result.status == expect else 1


if __name__ == "__main__":
    raise SystemExit(main())
