import numpy as np


def direction_vec(angle):
    return np.array([np.cos(angle), np.sin(angle)])


def vec_len(vec):
    return np.linalg.norm(vec)


def unit_vec(vec):
    return vec / vec_len(vec)