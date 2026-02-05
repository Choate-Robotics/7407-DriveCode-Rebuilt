import math

def bounded_angle_diff(theta_from: float, theta_too: float) -> float:
    """
    Finds the bounded (from -π to π) angle difference between two unbounded angles
    """
    res = math.fmod(theta_too - theta_from, 6.283185307179586)
    if res > math.pi:
        res -= 6.283185307179586
    if res < -math.pi:
        res += 6.283185307179586
    return res

def curve(x, d, c=1):
    if abs(x) < d:
        return 0
    elif x < 0:
        return -1 * math.pow((-1 * (x + d) / (1 - d)), c)
    return math.pow(((x - d) / (1 - d)), c)