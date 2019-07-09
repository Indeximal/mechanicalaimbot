import numpy as np

from mechbot.controller.device import VirtualDevice, StepperMotor
from mechbot.controller.calibration import CalibrationHelper

if __name__ == '__main__':
    # correct_motor1 = StepperMotor((2., 2.), 200, 1, -2.356)
    # correct_motor2 = StepperMotor((-2., 2.), 200, 1, -0.785)
    # correct_gap = .1
    initial_motor_1 = StepperMotor((1.8, 2.1), 200, 1, -2.51)
    initial_motor_2 = StepperMotor((-2.2, 1.94), 200, 1, -0.65)
    initial_device = VirtualDevice(initial_motor_1, initial_motor_2, .14, 0.)

    calibrator = CalibrationHelper(None, initial_device)

    calibration_points = [(np.array([ 0.01166689, -0.00912563]), 1, 0), (np.array([ 0.12092835, -0.10045304]), 3, 0), (np.array([ 0.27449883, -0.20465463]), 5, 0), (np.array([ 0.42277604, -0.2924636 ]), 7, 0), (np.array([ 0.54410514, -0.10222947]), 7, 4), (np.array([0.42123738, 0.00534241]), 5, 4), (np.array([0.29696039, 0.08145666]), 3, 4), (np.array([0.18568755, 0.19216418]), 1, 4), (np.array([0.15589698, 0.1986122 ]), -1, 4), (np.array([0.06525337, 0.26944048]), -3, 4), (np.array([-0.06651816,  0.36750731]), -5, 4), (np.array([-0.19252463,  0.4821675 ]), -7, 4), (np.array([-0.32384476,  0.61937707]), -9, 4), (np.array([-0.21181797,  0.66689663]), -9, 6), (np.array([3.99072985e-04, 7.96086106e-01]), -9, 9), (np.array([0.2016434 , 0.66733924]), -6, 9), (np.array([0.32044651, 0.61680363]), -2, 9), (np.array([0.46947578, 0.41343054]), 2, 8), (np.array([0.59391324, 0.23132401]), 5, 7), (np.array([0.77476793, 0.13195244]), 8, 7), (np.array([ 0.78896271, -0.1475068 ]), 10, 5), (np.array([ 0.66537986, -0.37880229]), 10, 1), (np.array([ 0.38573302, -0.72955928]), 9, -3), (np.array([ 0.20061834, -0.49018643]), 6, -3), (np.array([ 0.07290623, -0.33711895]), 4, -3), (np.array([-0.17871288, -0.17999766]), 1, -4), (np.array([-0.31176417, -0.02954987]), -3, -4), (np.array([-0.47263947,  0.20044825]), -6, -4), (np.array([-0.59588791,  0.33722084]), -8, -4), (np.array([-0.71232924,  0.1695367 ]), -7, -7), (np.array([-0.72397451, -0.10651083]), -5, -9), (np.array([-0.63786382, -0.28111557]), -2, -9), (np.array([-0.31516298, -0.47833242]), 2, -7), (np.array([-0.06694517, -0.67444261]), 5, -6), (np.array([-0.51671547, -0.69831482]), 2, -10), (np.array([-0.06073953, -0.49829855]), 4, -5), (np.array([-0.34573393, -0.24804404]), -1, -6), (np.array([-0.35006418,  0.27358424]), -6, -2), (np.array([-0.60092873,  0.56441755]), -10, -2)]

    calibrator.examples = calibration_points

    epsilon = 0.01
    for epoch in range(200):
        calibrator.gradient_descent_step(epsilon)
        loss = calibrator.calc_loss()
        if loss < .1:
            epsilon = 0.006
        if loss < .03:
            epsilon = 0.001

        if epoch % 1 != 0:
            continue
        print("{:>3} {:.5f} [{m1.pos[0]:.3f}|{m1.pos[1]:.3f}, {m2.pos[0]:.3f}|"
              "{m2.pos[1]:.3f}, {m1.align:.3f}, {m2.align:.3f}, {gap:.4f}]"
              .format(epoch, loss, m1=initial_motor_1, m2=initial_motor_2,
                      gap=initial_device.gap))
