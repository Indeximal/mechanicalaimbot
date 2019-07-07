import numpy as np


def direction_vec(angle):
    return np.array([np.cos(angle), np.sin(angle)])


def vec_len(vec):
    return np.linalg.norm(vec)


def unit_vec(vec):
    return vec / vec_len(vec)


def perp_vec2d(vec):
    return np.array((-vec[1], vec[0]))


def ray_intersect2d(ray1, ray2):
    # from stackoverflow.com/questions/3252194/numpy-and-line-intersections
    da = ray1[1]
    db = ray2[1]
    dp = ray1[0] - ray2[0]
    dap = perp_vec2d(da)
    denom = np.dot(dap, db)
    num = np.dot(dap, dp)
    return (num / denom.astype(float)) * db + ray2[0]
