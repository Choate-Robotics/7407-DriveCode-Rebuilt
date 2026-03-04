import math

def bounded_angle_diff(a, b):
    """
    Returns the bounded difference (a - b) wrapped to (-π, π].
    """
    res = (a - b + math.pi) % (2 * math.pi) - math.pi
    print(res)
    return res

def curve(x, d, c=1):
    if abs(x) < d:
        return 0
    elif x < 0:
        return -1 * math.pow((-1 * (x + d) / (1 - d)), c)
    return math.pow(((x - d) / (1 - d)), c)