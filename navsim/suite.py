import sys
from pathlib import Path

from navsim.runner import expectation_failures, format_result, load_scenario, run_scenario

ROOT = Path(__file__).resolve().parents[1]


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    folder = Path(args[0]) if args else ROOT / "scenarios"
    paths = sorted(folder.glob("*.yaml"))
    if not paths:
        print(f"没有场景: {folder}", file=sys.stderr)
        return 2
    failed = 0
    for path in paths:
        scenario = load_scenario(path)
        result = run_scenario(scenario)
        failures = expectation_failures(scenario, result)
        mark = "PASS" if not failures else "FAIL"
        print(f"{mark}  {format_result(scenario, result)}")
        for item in failures:
            print(f"      {item}")
        failed += int(bool(failures))
    print(f"{len(paths) - failed} passed, {failed} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
