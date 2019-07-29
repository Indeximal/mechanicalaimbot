import numpy as np
import unittest

from mechbot.utils.vector_utils import vec_len, dir_vec, unit_vec


class VectorTests(unittest.TestCase):
    def assertVecEqual(self, first, second):
        self.assertEqual(first.shape, second.shape)
        for a, b in zip(first.flat, second.flat):
            self.assertAlmostEqual(a, b)

    def test_dir_vec(self):
        angle = 0
        vec = np.array((1., 0.))
        self.assertVecEqual(dir_vec(angle), vec)

        angle = np.pi / 4
        vec = 1 / np.array((np.sqrt(2), np.sqrt(2)))
        self.assertVecEqual(dir_vec(angle), vec)

        angle = 3.282735  # random
        self.assertAlmostEqual(vec_len(dir_vec(angle)), 1)

        angle = .13  # random
        self.assertVecEqual(dir_vec(angle), dir_vec(angle + 2 * np.pi))

    def test_vec_len(self):
        vec = np.array((3., 0.))
        self.assertEqual(vec_len(vec), 3.)

        vec = np.array((3., 4.))
        self.assertEqual(vec_len(vec), 5.)

        vec = np.array((-2.2, 6.21))
        self.assertEqual(vec_len(vec), vec_len(-vec))

        vec = np.zeros(2)
        self.assertEqual(vec_len(vec), 0.)

    def test_unit_vec(self):
        vec = np.array((-2.43, 0.))
        vec2 = np.array((-1., 0.))
        self.assertVecEqual(unit_vec(vec), vec2)

        vec = np.array([0.77027559, 0.5521145])
        uvec = np.array([0.81277533, 0.58257726])
        self.assertVecEqual(unit_vec(vec), uvec)

        vec = np.array([52., 4.2])
        self.assertAlmostEqual(vec_len(unit_vec(vec)), 1.)

        with self.assertRaises(ZeroDivisionError):
            unit_vec(np.zeros(2))


# def perp_vec2d(vec):
#     return np.array((-vec[1], vec[0]))


# def ray_intersect2d(ray1, ray2):
#     # from stackoverflow.com/questions/3252194/numpy-and-line-intersections
#     da = ray1[1]
#     db = ray2[1]
#     dp = ray1[0] - ray2[0]
#     dap = perp_vec2d(da)
#     denom = np.dot(dap, db)
#     num = np.dot(dap, dp)
#     return (num / denom.astype(float)) * db + ray2[0]
