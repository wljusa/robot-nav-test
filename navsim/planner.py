import heapq
import math

from navsim.world import World


def plan(
    world: World,
    start: tuple[float, float],
    goal: tuple[float, float],
    resolution: float = 0.25,
    t: float = 0.0,
) -> list[tuple[float, float]] | None:
    nx = int(world.width / resolution)
    ny = int(world.height / resolution)
    if nx < 2 or ny < 2:
        return None

    def center(ix: int, iy: int) -> tuple[float, float]:
        return (ix + 0.5) * resolution, (iy + 0.5) * resolution

    def to_cell(x: float, y: float) -> tuple[int, int]:
        ix = min(nx - 1, max(0, int(x / resolution)))
        iy = min(ny - 1, max(0, int(y / resolution)))
        return ix, iy

    def blocked(ix: int, iy: int) -> bool:
        x, y = center(ix, iy)
        return world.body_collides(x, y, t)

    start_cell = to_cell(*start)
    goal_cell = to_cell(*goal)
    if blocked(*start_cell) or blocked(*goal_cell):
        return None

    offsets = [
        (1, 0, 1.0),
        (-1, 0, 1.0),
        (0, 1, 1.0),
        (0, -1, 1.0),
        (1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)),
        (-1, 1, math.sqrt(2)),
        (-1, -1, math.sqrt(2)),
    ]

    def neighbors(ix: int, iy: int):
        for dx, dy, cost in offsets:
            jx, jy = ix + dx, iy + dy
            if not (0 <= jx < nx and 0 <= jy < ny) or blocked(jx, jy):
                continue
            if dx != 0 and dy != 0 and (blocked(ix + dx, iy) or blocked(ix, iy + dy)):
                continue
            yield jx, jy, cost

    open_heap: list[tuple[float, tuple[int, int]]] = []
    heapq.heappush(open_heap, (0.0, start_cell))
    came: dict[tuple[int, int], tuple[int, int]] = {}
    cost = {start_cell: 0.0}

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == goal_cell:
            break
        for jx, jy, step_cost in neighbors(*current):
            nxt = (jx, jy)
            new_cost = cost[current] + step_cost
            if new_cost >= cost.get(nxt, math.inf):
                continue
            cost[nxt] = new_cost
            came[nxt] = current
            gx, gy = center(jx, jy)
            heuristic = math.hypot(gx - goal[0], gy - goal[1]) / resolution
            heapq.heappush(open_heap, (new_cost + heuristic, nxt))

    if goal_cell not in cost:
        return None

    cells = [goal_cell]
    while cells[-1] != start_cell:
        cells.append(came[cells[-1]])
    cells.reverse()
    path = [center(ix, iy) for ix, iy in cells]
    if path[-1] != goal and not world.body_collides(*goal, t):
        path.append(goal)
    return path
