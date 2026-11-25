import math
from dataclasses import dataclass, field

from navsim.planner import plan
from navsim.world import World

DT = 0.05
DT_NS = int(DT * 1e9)


@dataclass
class TopicMsg:
    stamp_ns: int
    data: dict


@dataclass
class RunResult:
    status: str
    collided: bool
    x: float
    y: float
    yaw: float
    goal_distance: float
    topics: dict[str, list[TopicMsg]] = field(default_factory=dict)

    def hz(self, topic: str) -> float:
        stamps = [msg.stamp_ns for msg in self.topics.get(topic, [])]
        if len(stamps) < 2:
            return 0.0
        duration = (stamps[-1] - stamps[0]) / 1e9
        if duration <= 0:
            return 0.0
        return (len(stamps) - 1) / duration

    def monotonic(self, topic: str) -> bool:
        stamps = [msg.stamp_ns for msg in self.topics.get(topic, [])]
        return all(b > a for a, b in zip(stamps, stamps[1:]))


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def run_world(
    world: World,
    start: tuple[float, float, float],
    goal: tuple[float, float],
    goal_tolerance: float,
    timeout_sec: float,
) -> RunResult:
    topics: dict[str, list[TopicMsg]] = {"/odom": [], "/cmd_vel": [], "/scan": []}
    sx, sy, yaw = start
    distance = math.hypot(sx - goal[0], sy - goal[1])
    path = plan(world, (sx, sy), goal)
    if path is None:
        return RunResult("blocked", False, sx, sy, yaw, distance, topics)

    x, y = sx, sy
    waypoint = 0
    collided = False
    steps = max(1, int(timeout_sec / DT))
    for step in range(steps):
        distance = math.hypot(x - goal[0], y - goal[1])
        if distance <= goal_tolerance:
            return RunResult("success", False, x, y, yaw, distance, topics)

        while waypoint < len(path) - 1 and math.hypot(x - path[waypoint][0], y - path[waypoint][1]) < 0.3:
            waypoint += 1
        tx, ty = path[waypoint]
        error = _wrap(math.atan2(ty - y, tx - x) - yaw)
        if abs(error) > 0.45:
            linear, angular = 0.0, math.copysign(1.2, error)
        else:
            linear, angular = 0.75, max(-1.2, min(1.2, error * 1.8))

        stamp = step * DT_NS
        next_yaw = _wrap(yaw + angular * DT)
        next_x = x + linear * math.cos(next_yaw) * DT
        next_y = y + linear * math.sin(next_yaw) * DT
        topics["/cmd_vel"].append(TopicMsg(stamp, {"linear": linear, "angular": angular}))
        topics["/odom"].append(TopicMsg(stamp, {"x": x, "y": y, "yaw": yaw}))
        if step % 2 == 0:
            topics["/scan"].append(
                TopicMsg(stamp, {"range": round(world.ray_range(x, y, yaw), 3)})
            )

        if world.body_collides(next_x, next_y):
            collided = True
            return RunResult("collision", collided, x, y, yaw, distance, topics)
        x, y, yaw = next_x, next_y, next_yaw

    distance = math.hypot(x - goal[0], y - goal[1])
    return RunResult("timeout", collided, x, y, yaw, distance, topics)
