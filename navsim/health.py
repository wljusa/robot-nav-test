from dataclasses import dataclass
from pathlib import Path

import yaml

from navsim.sim import TopicMsg

NS_PER_MS = 1_000_000
NS_PER_SEC = 1_000_000_000
SPEC_PATH = Path(__file__).resolve().parents[1] / "expectations" / "topics.yaml"


@dataclass(frozen=True)
class Finding:
    topic: str
    code: str
    detail: str

    def __str__(self) -> str:
        return f"{self.code}\t{self.topic}\t{self.detail}"


def load_topic_spec(path: Path = SPEC_PATH) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or "topics" not in data:
        raise ValueError(f"话题期望格式错误: {path}")
    return data


def check_stamps(topic: str, messages: list[TopicMsg], spec: dict) -> list[Finding]:
    findings: list[Finding] = []
    stamps = [msg.stamp_ns for msg in messages]
    if len(stamps) < 2:
        findings.append(Finding(topic, "count", f"有效消息 {len(stamps)} 条，无法计算频率"))
        return findings

    beams = spec.get("beams")
    if beams is not None:
        bad = [len(msg.data.get("ranges", [])) for msg in messages if len(msg.data.get("ranges", [])) != beams]
        if bad:
            findings.append(Finding(topic, "beams", f"激光束数不是 {beams}"))

    previous = stamps[0]
    max_gap_ms = 0.0
    monotonic = True
    for stamp in stamps[1:]:
        if stamp <= previous:
            monotonic = False
        else:
            max_gap_ms = max(max_gap_ms, (stamp - previous) / NS_PER_MS)
            previous = stamp
    if not monotonic:
        findings.append(Finding(topic, "monotonic", "时间戳相对记录顺序出现回退"))

    limit = float(spec["max_gap_ms"])
    if max_gap_ms > limit:
        findings.append(Finding(topic, "gap", f"最大间隔 {max_gap_ms:.1f}ms，阈值 {limit:.1f}ms"))

    duration = (stamps[-1] - stamps[0]) / NS_PER_SEC
    if duration <= 0:
        findings.append(Finding(topic, "hz", "时间跨度为 0"))
        return findings
    measured = (len(stamps) - 1) / duration
    expected = float(spec["hz"])
    tolerance = float(spec["hz_tolerance"])
    if abs(measured - expected) / expected > tolerance:
        findings.append(
            Finding(topic, "hz", f"实测 {measured:.2f}Hz，期望 {expected:.2f}Hz，容差 {tolerance:.0%}")
        )
    return findings


def check_topics(topics: dict[str, list[TopicMsg]], spec: dict | None = None) -> list[Finding]:
    spec = load_topic_spec() if spec is None else spec
    findings: list[Finding] = []
    present: dict[str, list[TopicMsg]] = {}
    for topic, rule in spec["topics"].items():
        messages = topics.get(topic) or []
        if not messages:
            findings.append(Finding(topic, "missing", "话题没有消息"))
            continue
        present[topic] = messages
        findings.extend(check_stamps(topic, messages, rule))

    if len(present) >= 2:
        starts = {topic: messages[0].stamp_ns for topic, messages in present.items()}
        skew_ms = (max(starts.values()) - min(starts.values())) / NS_PER_MS
        limit = float(spec.get("sync_max_start_skew_ms", 50))
        if skew_ms > limit:
            findings.append(Finding("*", "skew", f"话题首帧相差 {skew_ms:.1f}ms，阈值 {limit:.1f}ms"))
    return findings
