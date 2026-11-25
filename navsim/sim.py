import math
from dataclasses import dataclass, field

from navsim.planner import plan
from navsim.sensors import SCAN_EVERY, imu_sample, laser_scan
from navsim.world import World

DT = 0.05
DT_NS = int(DT * 1e9)
REPLAN_STEPS = 5


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
    path_length: float = 0.0
    min_clearance: float = math.inf
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

    def max_linear(self) -> float:
        values = [msg.data["linear"] for msg in self.topics.get("/cmd_vel", [])]
        return max(values) if values else 0.0

    def max_abs_angular(self) -> float:
        values = [abs(msg.data["angular"]) for msg in self.topics.get("/cmd_vel", [])]
        return max(values) if values else 0.0


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _finish(status: str, collided: bool, x: float, y: float, yaw: float, goal, path_length, clearance, topics) -> RunResult:
    return RunResult(
        status,
        collided,
        x,
        y,
        yaw,
        math.hypot(x - goal[0], y - goal[1]),
        path_length,
        clearance,
        topics,
    )


def run_world(
    world: World,
    start: tuple[float, float, float],
    goal: tuple[float, float],
    goal_tolerance: float,
    timeout_sec: float,
) -> RunResult:
    topics: dict[str, list[TopicMsg]] = {"/odom": [], "/cmd_vel": [], "/scan": [], "/imu": []}
    x, y, yaw = start
    path: list[tuple[float, float]] | None = None
    waypoint = 0
    next_replan = 0
    previous_linear = 0.0
    path_length = 0.0
    clearance = math.inf
    steps = max(1, int(timeout_sec / DT))

    for step in range(steps):
        t = step * DT
        if math.hypot(x - goal[0], y - goal[1]) <= goal_tolerance:
            return _finish("success", False, x, y, yaw, goal, path_length, clearance, topics)

        if step >= next_replan or path is None:
            path = plan(world, (x, y), goal, t=t)
            waypoint = 0
            next_replan = step + REPLAN_STEPS
            if path is None:
                return _finish("blocked", False, x, y, yaw, goal, path_length, clearance, topics)

        while waypoint < len(path) - 1 and math.hypot(x - path[waypoint][0], y - path[waypoint][1]) < 0.3:
            waypoint += 1
        tx, ty = path[waypoint]
        error = _wrap(math.atan2(ty - y, tx - x) - yaw)
        if abs(error) > 0.45:
            linear, angular = 0.0, math.copysign(1.2, error)
        else:
            linear, angular = 0.75, max(-1.2, min(1.2, error * 1.8))

        stamp = step * DT_NS
        clearance = min(clearance, world.clearance(x, y, t))
        topics["/cmd_vel"].append(TopicMsg(stamp, {"linear": linear, "angular": angular}))
        topics["/odom"].append(TopicMsg(stamp, {"x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 4)}))
        topics["/imu"].append(TopicMsg(stamp, imu_sample(angular, linear, previous_linear)))
        if step % SCAN_EVERY == 0:
            topics["/scan"].append(TopicMsg(stamp, laser_scan(world, x, y, yaw, t)))

        next_yaw = _wrap(yaw + angular * DT)
        next_x = x + linear * math.cos(next_yaw) * DT
        next_y = y + linear * math.sin(next_yaw) * DT
        if world.body_collides(next_x, next_y, t):
            return _finish("collision", True, x, y, yaw, goal, path_length, clearance, topics)
        path_length += math.hypot(next_x - x, next_y - y)
        x, y, yaw = next_x, next_y, next_yaw
        previous_linear = linear

    return _finish("timeout", False, x, y, yaw, goal, path_length, clearance, topics)
