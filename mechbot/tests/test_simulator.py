import unittest

from mechbot.controller.simulator import MechanicalSimulator
from mechbot.controller.device import VirtualDevice, StepperMotor


class SimulatorTests(unittest.TestCase):
    def setUp(self):
        correct_motor1 = StepperMotor((2., 2.), 200, 1, -2.356)
        correct_motor2 = StepperMotor((-2., 2.), 200, 1, -0.785)
        correct_device = VirtualDevice(correct_motor1, correct_motor2, .1, .0)
        self.device = correct_device

    # Only test whether methods exist, don't bother testing actual simulation
    # Implementation is somewhat tested in test_calibration
    def test_methods(self):
        m = MechanicalSimulator(self.device, .1)

        m.loop()
        m.tick(.02)

        interface = m.get_interface()

        pos = interface.get_input()
        self.assertEqual(pos.shape, (2,))

        interface.cmd_goto(1, 1)
