import json
from pathlib import Path

from navsim.sim import TopicMsg


def write_log(path: Path, topics: dict[str, list[TopicMsg]]) -> None:
    lines = []
    for topic, messages in topics.items():
        for msg in messages:
            lines.append({"topic": topic, "stamp_ns": msg.stamp_ns, "data": msg.data})
    lines.sort(key=lambda item: (item["stamp_ns"], item["topic"]))
    path.write_text("".join(json.dumps(item, ensure_ascii=False) + "\n" for item in lines), encoding="utf-8")


def read_log(path: Path) -> dict[str, list[TopicMsg]]:
    topics: dict[str, list[TopicMsg]] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        item = json.loads(line)
        topics.setdefault(item["topic"], []).append(TopicMsg(int(item["stamp_ns"]), item["data"]))
    return topics
