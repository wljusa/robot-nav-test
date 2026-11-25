import math

BEAMS = 16
ANGLE_MIN = -math.pi / 2
ANGLE_MAX = math.pi / 2
SCAN_EVERY = 2


def laser_scan(world, x: float, y: float, yaw: float, t: float) -> dict:
    ranges = []
    step = (ANGLE_MAX - ANGLE_MIN) / (BEAMS - 1)
    for index in range(BEAMS):
        bearing = yaw + ANGLE_MIN + step * index
        ranges.append(round(world.ray_range(x, y, bearing, t), 3))
    return {
        "angle_min": round(ANGLE_MIN, 4),
        "angle_max": round(ANGLE_MAX, 4),
        "beams": BEAMS,
        "ranges": ranges,
    }


def imu_sample(angular: float, linear: float, previous_linear: float) -> dict:
    return {
        "gyro_z": round(angular, 4),
        "accel_x": round((linear - previous_linear) / 0.05, 4),
    }
