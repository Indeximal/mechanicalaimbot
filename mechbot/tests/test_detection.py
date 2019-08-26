import unittest

import numpy as np

from mechbot.inference.detection import ObjectDetector
from mechbot import resources


class ObjectDetectorTests(unittest.TestCase):
    def test_load_and_run_images(self):
        expected_boxes = np.array([[[0.4448856, 0.36333707, 0.76647425, 0.46323308], [0.46954894, 0.21764049, 0.7389226, 0.30699983], [0.46683833, 0.23715134, 0.52663296, 0.26393786], [0.45101032, 0.40322357, 0.519443 , 0.43210453], [0., 0., 0., 0.]], [[0.47604874, 0.7008276, 0.5227535, 0.72165835], [0.4775463, 0.68523484, 0.6825288, 0.7479282], [0.4832109, 0.46942207, 0.5305029, 0.49177352], [0.44525865, 0.4576475, 0.7181225, 0.5434944], [0., 0., 0., 0.]]], dtype=np.float32)
        expected_scores = np.array([[0.99999994, 0.99999744, 0.9999924, 0.9998251, 0.], [0.9999981, 0.9998213, 0.99811554, 0.79213023, 0.]], dtype=np.float32)
        expected_classes = np.array([[1, 1, 2, 2, 1], [2, 1, 2, 1, 1]])

        with ObjectDetector(resources.FROZEN_DETECTION_GRAPH) as detector:
            result = detector.load_and_run_images(resources.TEST_FRAMES[:2])
            boxes, scores, classes = result

            self.assertTrue(np.allclose(expected_boxes, boxes[:, :5]))
            self.assertTrue(np.allclose(expected_scores, scores[:, :5]))
            self.assertTrue(np.allclose(expected_classes, classes[:, :5]))

            # print(repr(boxes[:, :5]), repr(scores[:, :5]), repr(classes[:, :5]))
