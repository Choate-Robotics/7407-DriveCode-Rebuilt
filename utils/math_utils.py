import math

def bounded_angle_diff(a, b):
    """
    Returns the bounded difference (a - b) wrapped to (-π, π].
    """
    res = (a - b + math.pi) % (2 * math.pi) - math.pi
    return res

def curve(x, d, c=1):
    if abs(x) < d:
        return 0
    elif x < 0:
        return -1 * math.pow((-1 * (x + d) / (1 - d)), c)
    return math.pow(((x - d) / (1 - d)), c)

def radial_curve(x: float, y: float, d_min: float, d_max: float = 1.0, c: float = 1) -> tuple[float, float]:
    """
    Curves a 2D joystick input as one vector instead of per-axis, so the deadzone
    is circular and direction is preserved near the edges - matches Unity's default
    StickDeadzone processor (used by MoSim's translate input).
    """
    magnitude = math.hypot(x, y)
    if magnitude < d_min:
        return 0.0, 0.0

    clamped = min(magnitude, d_max)
    scaled = math.pow((clamped - d_min) / (d_max - d_min), c)

    return (x / magnitude) * scaled, (y / magnitude) * scaled
