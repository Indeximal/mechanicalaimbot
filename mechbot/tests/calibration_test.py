import numpy as np

from mechbot.controller.calibration import CalibrationHelper

if __name__ == '__main__':
    calibrator = CalibrationHelper(None)
    calibrator.m1_points = [(5, np.array([ 0.28463742, -0.21123531])), (4, np.array([ 0.20063939, -0.15300247])), (3, np.array([ 0.13333687, -0.10863872])), (2, np.array([ 0.05386075, -0.04736226])), (1, np.array([ 0.00880334, -0.00774118])), (0, np.array([ 0.00880334, -0.00774118])), (-1, np.array([0.00056772, 0.00102885])), (-2, np.array([-0.05560635,  0.06308939])), (-3, np.array([-0.10890592,  0.13116933])), (-4, np.array([-0.15147622,  0.19282695])), (-5, np.array([-0.19838517,  0.26693928]))]
    calibrator.m2_points = [(5, np.array([0.19877921, 0.26745531])), (4, np.array([0.15501349, 0.20299395])), (3, np.array([0.11231911, 0.1377924 ])), (2, np.array([0.0550211 , 0.06257468])), (1, np.array([0.00879827, 0.01000615])), (0, np.array([0.00879827, 0.01000615])), (-1, np.array([-4.14783218e-05,  1.70506646e-03])), (-2, np.array([-0.06217002, -0.05479352])), (-3, np.array([-0.13054194, -0.10837958])), (-4, np.array([-0.19233082, -0.15108058])), (-5, np.array([-0.28459116, -0.21119368]))]

    m1, m2 = calibrator.compute_guess(200)
    print(m1)  # correct: -2.3561944902, (2, 2), .1
    print(m2)  # correct: -0.7853981634, (-2, 2), .1
