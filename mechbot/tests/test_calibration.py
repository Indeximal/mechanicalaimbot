import unittest

import numpy as np

from mechbot.controller.device import VirtualDevice, StepperMotor
from mechbot.controller.calibration import GradientDescentHelper, CalibrationHelper
from mechbot.controller.simulator import MechanicalSimulator
from mechbot.utils.vector_utils import vec_len


def device_dist(first, second):
    def error(a, b):
        return abs(a - b) / max(abs(a), abs(b))

    m1x = error(first.motor1.pos[0], second.motor1.pos[0])
    m1y = error(first.motor1.pos[1], second.motor1.pos[1])
    m2x = error(first.motor2.pos[0], second.motor2.pos[0])
    m2y = error(first.motor2.pos[1], second.motor2.pos[1])
    a1 = error(first.motor1.align, second.motor1.align)
    a2 = error(first.motor2.align, second.motor2.align)
    gp = error(first.gap - first.stick, second.gap - second.stick) * .3

    error_vec = [m1x, m2x, m1y, m2y, a1, a2, gp]
    return vec_len(error_vec)


class GradientDescentTests(unittest.TestCase):
    def test_raw_descent(self):
        correct_motor1 = StepperMotor((2., 2.), 200, 1, -2.356)
        correct_motor2 = StepperMotor((-2., 2.), 200, 1, -0.785)
        correct_device = VirtualDevice(correct_motor1, correct_motor2, .1, .0)

        initial_m1 = StepperMotor((1.8, 2.1), 200, 1, -2.51)
        initial_m2 = StepperMotor((-2.2, 1.94), 200, 1, -0.72)
        initial_device = VirtualDevice(initial_m1, initial_m2, .12, 0.)


        calibration_points = [(np.array([0.01166689, -0.00912563]), 1, 0), (np.array([0.12092835, -0.10045304]), 3, 0), (np.array([0.27449883, -0.20465463]), 5, 0), (np.array([0.42277604, -0.2924636]), 7, 0), (np.array([0.54410514, -0.10222947]), 7, 4), (np.array([0.42123738, 0.00534241]), 5, 4), (np.array([0.29696039, 0.08145666]), 3, 4), (np.array([0.18568755, 0.19216418]), 1, 4), (np.array([0.15589698, 0.1986122]), -1, 4), (np.array([0.06525337, 0.26944048]), -3, 4), (np.array([-0.06651816,  0.36750731]), -5, 4), (np.array([-0.19252463,  0.4821675]), -7, 4), (np.array([-0.32384476,  0.61937707]), -9, 4), (np.array([-0.21181797,  0.66689663]), -9, 6), (np.array([3.99072985e-04, 7.96086106e-01]), -9, 9), (np.array([0.2016434 , 0.66733924]), -6, 9), (np.array([0.32044651, 0.61680363]), -2, 9), (np.array([0.46947578, 0.41343054]), 2, 8), (np.array([0.59391324, 0.23132401]), 5, 7), (np.array([0.77476793, 0.13195244]), 8, 7), (np.array([0.78896271, -0.1475068]), 10, 5), (np.array([0.66537986, -0.37880229]), 10, 1), (np.array([0.38573302, -0.72955928]), 9, -3), (np.array([0.20061834, -0.49018643]), 6, -3), (np.array([0.07290623, -0.33711895]), 4, -3), (np.array([-0.17871288, -0.17999766]), 1, -4), (np.array([-0.31176417, -0.02954987]), -3, -4), (np.array([-0.47263947,  0.20044825]), -6, -4), (np.array([-0.59588791,  0.33722084]), -8, -4), (np.array([-0.71232924,  0.1695367]), -7, -7), (np.array([-0.72397451, -0.10651083]), -5, -9), (np.array([-0.63786382, -0.28111557]), -2, -9), (np.array([-0.31516298, -0.47833242]), 2, -7), (np.array([-0.06694517, -0.67444261]), 5, -6), (np.array([-0.51671547, -0.69831482]), 2, -10), (np.array([-0.06073953, -0.49829855]), 4, -5), (np.array([-0.34573393, -0.24804404]), -1, -6), (np.array([-0.35006418,  0.27358424]), -6, -2), (np.array([-0.60092873,  0.56441755]), -10, -2)]

        helper = GradientDescentHelper(initial_device, calibration_points)

        initial_diff = device_dist(correct_device, initial_device)
        L_0 = helper.calc_loss()
        self.assertIsInstance(L_0, float)


        epsilon = 0.005
        for epoch in range(20):
            helper.gradient_descent_step(epsilon)
            # loss = helper.calc_loss()

            # # print every N epochs
            # if epoch % 1 != 0:
            #     continue
            # print("{:>3} {:.5f} [{m1.pos[0]:.3f}|{m1.pos[1]:.3f}, {m2.pos[0]:.3f}|"
            #       "{m2.pos[1]:.3f}, {m1.align:.3f}, {m2.align:.3f}, {gap:.4f}]"
            #       .format(epoch, loss, m1=initial_m1, m2=initial_m2,
            #               gap=initial_device.gap))

        end_diff = device_dist(correct_device, initial_device)
        L_1 = helper.calc_loss()

        # Loss should decrease and devices should get more similar
        self.assertLess(L_1, L_0)
        self.assertLess(end_diff, initial_diff)


class CalibrationHelperTests(unittest.TestCase):
    def test_compute_guess(self):
        correct_motor1 = StepperMotor((2., 2.), 200, 1, -2.356)
        correct_motor2 = StepperMotor((-2., 2.), 200, 1, -0.785)
        correct_device = VirtualDevice(correct_motor1, correct_motor2, .1, .0)

        m1 = [(np.array([0.27160691, -0.20317596]), 5, 0), (np.array([0.20676309, -0.15742282]), 4, 0), (np.array([0.13271406, -0.10686997]), 3, 0), (np.array([0.06263169, -0.054815]), 2, 0), (np.array([0.01164888, -0.01021449]), 1, 0), (np.array([0.01164888, -0.01021449]), 0, 0), (np.array([0.00325921, -0.0012804]), -1, 0), (np.array([-0.04638552, 0.0527001]), -2, 0), (np.array([-0.10352764, 0.12450347]), -3, 0), (np.array([-0.15328752, 0.19400233]), -4, 0), (np.array([-0.20298883, 0.27140088]), -5, 0)]
        m2 = [(np.array([0.20389675, 0.27254819]), 0, 5), (np.array([0.15808902, 0.20762993]), 0, 4), (np.array([0.10762277, 0.13363616]), 0, 3), (np.array([0.05551032, 0.0634259]), 0, 2), (np.array([0.01086852, 0.01239506]), 0, 1), (np.array([0.01086852, 0.01239506]), 0, 0), (np.array([0.00189124, 0.00396483]), 0, -1), (np.array([-0.0519427, -0.04571178]), 0, -2), (np.array([-0.12420106, -0.10326491]), 0, -3), (np.array([-0.19339693, -0.15281448]), 0, -4), (np.array([-0.270827, -0.20255798]), 0, -5)]
        circ = [(np.array([-0.270827, -0.20255798]), 0, -5), (np.array([0.78330786, -0.0104949]), 9, 6), (np.array([0.71644428, 0.28327746]), 6, 8), (np.array([0.43684435, 0.63909126]), -1, 10), (np.array([0.27960517, 0.71622122]), -6, 10), (np.array([-0.00384975, 0.79686061]), -9, 9), (np.array([-0.26240648, 0.73585103]), -10, 5), (np.array([-0.54334301, 0.49005728]), -9, -2), (np.array([-0.70535647, 0.28135708]), -8, -6), (np.array([-0.72747749, -0.09313065]), -5, -9), (np.array([-0.73373885, -0.26191317]), -3, -10), (np.array([-0.49437315, -0.5123216]), 1, -9), (np.array([-0.25732013, -0.74513801]), 4, -8), (np.array([0.05732964, -0.67265969]), 6, -5), (np.array([0.37859363, -0.55400031]), 8, -2), (np.array([0.64234298, -0.44001751]), 10, 1), (np.array([0.74098904, -0.24831318]), 10, 4), (np.array([0.4851849, 0.06526668]), 5, 5), (np.array([0.26240001, 0.3524004]), -2, 6), (np.array([-0.06208151, 0.49030003]), -6, 5), (np.array([-0.35875858, 0.28829828]), -6, -2), (np.array([-0.43349861, -0.13035942]), -3, -6), (np.array([-0.31149775, -0.31264366]), 1, -6), (np.array([0.11654927, -0.41631274]), 5, -3), (np.array([0.37485375, -0.21651425]), 6, 2)]
        all_points = m1 + m2 + circ

        # L_baseline = GradientDescentHelper(correct_device, all_points).calc_loss()
        # print(L_baseline)

        # Fake a object after point gathering
        calibrator = CalibrationHelper(None, 200)
        calibrator.m1_points = m1
        calibrator.m2_points = m2

        # compute a guess
        guess = calibrator.compute_guess()
        self.assertIsInstance(guess, VirtualDevice)
        calibrator.device_guess = guess

        # See how different the devices are
        device_diff = device_dist(guess, correct_device)
        self.assertLess(device_diff, .05)

        # See whether gradient descent improves results
        loss_helper = GradientDescentHelper(guess, all_points)
        L_0 = loss_helper.calc_loss()

        calibrator.optimize_guess(15, 0.001)
        L_1 = loss_helper.calc_loss()

        # Code to debug gradient descent
        # loss = helper.calc_loss()
        # print("{:>3} {:.5} [{m1.pos[0]:.3f}|{m1.pos[1]:.3f}, {m2.pos[0]:.3f}|"
        #       "{m2.pos[1]:.3f}, {m1.align:.3f}, {m2.align:.3f}, {gap:.4f}]"
        #       .format(epoch, loss, m1=self.device_guess.motor1, m2=self.device_guess.motor2,
        #               gap=self.device_guess.gap))

        self.assertLess(L_1, L_0)

    def test_full_calibration(self):
        correct_motor1 = StepperMotor((1.6, -1.4), 200, 1, 2.44)
        correct_motor2 = StepperMotor((-1.7, -1.5), 200, 1, 0.76)
        # correct_motor1 = StepperMotor((2., 2.), 200, 1, -2.356)
        # correct_motor2 = StepperMotor((-2., 2.), 200, 1, -0.785)
        correct_device = VirtualDevice(correct_motor1, correct_motor2, .13, .0)

        simulation = MechanicalSimulator(correct_device, .1)

        calibrator = CalibrationHelper(simulation.get_interface(), 200, 
                                       wait_duration=.001) # speed up time
        calibrator.start()

        tick = 0
        # Takes about 40k ticks or 1 seconds on my macbook air
        while not calibrator.is_done():
            calibrator.tick()
            simulation.tick(0.02)
            if tick % 4 == 0:
                simulation.loop()
            tick += 1

        # print(tick)
        # print("[{m1.pos[0]:.3f}|{m1.pos[1]:.3f}, {m2.pos[0]:.3f}|"
        #       "{m2.pos[1]:.3f}, {m1.align:.3f}, {m2.align:.3f}, {gap:.4f}]"
        #       .format(m1=guess.motor1, m2=guess.motor2, gap=guess.gap))

        guess = calibrator.get_result()
        self.assertLess(device_dist(correct_device, guess), .2)
        self.assertLess(calibrator.calculate_loss(), .06)
