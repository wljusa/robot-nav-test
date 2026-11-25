from dataclasses import dataclass


@dataclass(frozen=True)
class Rect:
    x: float
    y: float
    w: float
    h: float

    def contains_point(self, px: float, py: float) -> bool:
        return self.x <= px <= self.x + self.w and self.y <= py <= self.y + self.h

    def distance_to_point(self, px: float, py: float) -> float:
        nx = min(max(px, self.x), self.x + self.w)
        ny = min(max(py, self.y), self.y + self.h)
        return ((px - nx) ** 2 + (py - ny) ** 2) ** 0.5


@dataclass(frozen=True)
class MovingRect:
    x: float
    y: float
    w: float
    h: float
    vx: float = 0.0
    vy: float = 0.0

    def at(self, t: float) -> Rect:
        return Rect(self.x + self.vx * t, self.y + self.vy * t, self.w, self.h)
