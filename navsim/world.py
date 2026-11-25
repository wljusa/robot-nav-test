import math
from dataclasses import dataclass, field

from navsim.geom import MovingRect, Rect


@dataclass
class World:
    width: float
    height: float
    obstacles: list[Rect]
    robot_radius: float
    motions: list[MovingRect] = field(default_factory=list)

    def rects(self, t: float) -> list[Rect]:
        return [*self.obstacles, *(motion.at(t) for motion in self.motions)]

    def out_of_bounds(self, x: float, y: float) -> bool:
        return x < 0 or y < 0 or x > self.width or y > self.height

    def hits_obstacle(self, x: float, y: float, t: float) -> bool:
        return any(rect.contains_point(x, y) for rect in self.rects(t))

    def body_collides(self, x: float, y: float, t: float = 0.0) -> bool:
        if (
            x < self.robot_radius
            or y < self.robot_radius
            or x > self.width - self.robot_radius
            or y > self.height - self.robot_radius
        ):
            return True
        return any(rect.distance_to_point(x, y) <= self.robot_radius for rect in self.rects(t))

    def clearance(self, x: float, y: float, t: float) -> float:
        bound = min(x, y, self.width - x, self.height - y) - self.robot_radius
        gaps = [rect.distance_to_point(x, y) - self.robot_radius for rect in self.rects(t)]
        return min([bound, *gaps])

    def ray_range(
        self,
        x: float,
        y: float,
        yaw: float,
        t: float,
        max_range: float = 6.0,
        step: float = 0.05,
    ) -> float:
        distance = 0.0
        while distance < max_range:
            distance = min(max_range, distance + step)
            px = x + math.cos(yaw) * distance
            py = y + math.sin(yaw) * distance
            if self.out_of_bounds(px, py) or self.hits_obstacle(px, py, t):
                return distance
        return max_range
