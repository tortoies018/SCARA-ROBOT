import math

STEPS_PER_REV = 200
MICROSTEPS = 8
PULLEY_RATIO = 5
STEPS_PER_JOINT_REV = STEPS_PER_REV * MICROSTEPS * PULLEY_RATIO

ARM1_LENGTH = 150.0
ARM2_LENGTH = 100.0

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi


def angle_to_steps(angle_deg: float) -> int:
    return int(round(angle_deg * STEPS_PER_JOINT_REV / 360.0))


def steps_to_angle(steps: int) -> float:
    return steps * 360.0 / STEPS_PER_JOINT_REV


def forward_kinematics(theta1_deg: float, theta2_deg: float) -> tuple[float, float]:
    t1 = theta1_deg * DEG2RAD
    t2 = theta2_deg * DEG2RAD
    x = ARM1_LENGTH * math.cos(t1) + ARM2_LENGTH * math.cos(t1 + t2)
    y = ARM1_LENGTH * math.sin(t1) + ARM2_LENGTH * math.sin(t1 + t2)
    return x, y


def inverse_kinematics(x: float, y: float) -> tuple[float, float] | None:
    d_sq = x * x + y * y
    l1_sq = ARM1_LENGTH * ARM1_LENGTH
    l2_sq = ARM2_LENGTH * ARM2_LENGTH
    cos_t2 = (d_sq - l1_sq - l2_sq) / (2.0 * ARM1_LENGTH * ARM2_LENGTH)
    if cos_t2 < -1.0 or cos_t2 > 1.0:
        return None
    sin_t2 = -math.sqrt(1.0 - cos_t2 * cos_t2)
    t2 = math.atan2(sin_t2, cos_t2)
    k1 = ARM1_LENGTH + ARM2_LENGTH * math.cos(t2)
    k2 = ARM2_LENGTH * math.sin(t2)
    t1 = math.atan2(y, x) - math.atan2(k2, k1)
    return t1 * RAD2DEG, t2 * RAD2DEG


def cartesian_to_steps(x: float, y: float) -> tuple[int, int] | None:
    ik = inverse_kinematics(x, y)
    if ik is None:
        return None
    t1, t2 = ik
    s1 = angle_to_steps(t1)
    s2 = angle_to_steps(t2)
    return s1, s2


def steps_to_cartesian(s1: int, s2: int) -> tuple[float, float]:
    t1 = steps_to_angle(s1)
    t2 = steps_to_angle(s2)
    return forward_kinematics(t1, t2)


def interpolate_line(x1: float, y1: float, x2: float, y2: float,
                     step_mm: float = 1.0) -> list[tuple[float, float]]:
    dx = x2 - x1
    dy = y2 - y1
    dist = math.hypot(dx, dy)
    if dist < 0.01:
        return [(x1, y1)]
    n = max(1, int(dist / step_mm))
    pts = []
    for i in range(n + 1):
        t = i / n
        pts.append((x1 + dx * t, y1 + dy * t))
    return pts


def interpolate_arc(x1: float, y1: float, x2: float, y2: float,
                    cx: float, cy: float, ccw: bool = True,
                    step_mm: float = 1.0) -> list[tuple[float, float]] | None:
    r1 = math.hypot(x1 - cx, y1 - cy)
    r2 = math.hypot(x2 - cx, y2 - cy)
    r = (r1 + r2) / 2.0
    if r < 0.1:
        return None
    a1 = math.atan2(y1 - cy, x1 - cx)
    a2 = math.atan2(y2 - cy, x2 - cx)
    if ccw:
        while a2 < a1:
            a2 += 2.0 * math.pi
    else:
        while a2 > a1:
            a2 -= 2.0 * math.pi
    arc_len = abs(a2 - a1) * r
    n = max(1, int(arc_len / step_mm))
    pts = []
    for i in range(n + 1):
        t = i / n
        a = a1 + (a2 - a1) * t
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def deg_per_sec_to_hz(deg_s: float) -> int:
    return int(round(deg_s * STEPS_PER_JOINT_REV / 360.0))


def hz_to_deg_per_sec(hz: int) -> float:
    return hz * 360.0 / STEPS_PER_JOINT_REV


def dda_interpolate_segments(points: list[tuple[float, float]],
                             speed_deg_s: float = 200) -> list[dict]:
    speed_hz = deg_per_sec_to_hz(speed_deg_s)
    segments = []
    for i in range(len(points) - 1):
        x1, y1 = points[i]
        x2, y2 = points[i + 1]
        ik1 = cartesian_to_steps(x1, y1)
        ik2 = cartesian_to_steps(x2, y2)
        if ik1 is None or ik2 is None:
            continue
        s1_1, s2_1 = ik1
        s1_2, s2_2 = ik2
        d1 = s1_2 - s1_1
        d2 = s2_2 - s2_1
        if d1 == 0 and d2 == 0:
            continue
        segments.append({
            "d1": d1, "d2": d2,
            "speed": speed_hz,
            "x1": x1, "y1": y1, "x2": x2, "y2": y2
        })
    return segments
