import time
from collections import deque

import numpy as np

from mechbot.utils.vector_utils import vec_len

# from mechbot.controller.simulator import MechanicalSimulator


class NotReadyError(Exception):
    pass


class MovementStateEnum:
    Idle = 1
    Waiting = 2
    Moving = 3


class MoveController:
    def __init__(self, interface):
        # Settings
        self.motion_threshold = .05  # units
        self.motion_path = deque(maxlen=20)  # ticks
        self.wait_duration = .7  # seconds

        self.interface = interface
        self.move_state = MovementStateEnum.Idle

    def move_to(self, s1, s2, callback):
        if self.move_state != MovementStateEnum.Idle:
            raise NotReadyError()

        self.initial_pos = self.interface.get_input()
        self.initial_time = time.time()
        self.callback = callback
        self.steps = (s1, s2)
        self.interface.cmd_goto(s1, s2)
        self.move_state = MovementStateEnum.Waiting

    def _finish(self):
        dt = time.time() - self.initial_time
        self.move_state = MovementStateEnum.Idle
        # Callback might invoke another move_to, so state must be idle before
        self.callback(self.steps, self.interface.get_input(), dt)

    def tick(self):
        # Don't do anything while in idle
        if self.move_state == MovementStateEnum.Idle:
            return

        pos = self.interface.get_input()
        if self.move_state == MovementStateEnum.Waiting:
            diff = vec_len(pos - self.initial_pos)
            # Has the stick moved?
            if diff > self.motion_threshold:
                self.move_state = MovementStateEnum.Moving
                self.motion_path.clear()
            # Has too much time passed?
            if (time.time() - self.initial_time) > self.wait_duration:
                self._finish()

        if self.move_state == MovementStateEnum.Moving:
            self.motion_path.append(pos)
            if len(self.motion_path) == self.motion_path.maxlen:
                # Minimal time waited
                diff = vec_len(self.motion_path[0] - self.motion_path[-1])
                if diff < self.motion_threshold:
                    # Stick is motionless
                    self._finish()


class CalibrationStateEnum:
    M1_Prepare = 1
    M1_Gather = 2
    M2_Prepare = 3


class CalibrationHelper(MoveController):
    """This class takes controll of an interface to approximate the device"""
    def __init__(self, interface):
        # Init
        MoveController.__init__(self, interface)
        self.m1_points = []
        # Settings
        self.max_deflection = .3

    def _get_deflection(self):
        return vec_len(self.interface.get_input())

    def _act(self):
        def reaction(steps, pos, t):
            over_deflection = vec_len(pos) > self.max_deflection

            if self.state == CalibrationStateEnum.M1_Prepare:
                if over_deflection:
                    self.state = CalibrationStateEnum.M1_Gather

            if self.state == CalibrationStateEnum.M1_Gather:
                self.m1_points.append((steps[0], pos))
                if self.step_1 < 0 and over_deflection:
                    self.state = CalibrationStateEnum.M2_Prepare

            self._act()

        if self.state == CalibrationStateEnum.M1_Prepare:
            self.step_1 += 1
            self.move_to(self.step_1, 0, reaction)

        if self.state == CalibrationStateEnum.M1_Gather:
            self.step_1 -= 1
            self.move_to(self.step_1, 0, reaction)

    def start(self):
        self.state = CalibrationStateEnum.M1_Prepare
        self.step_1 = 0
        self.step_2 = 0
        self._act()


class GradientDescentHelper:
    def __init__(self, device):
        self.device = device
        self.examples = []

    def add_example(self, step1, step2, pos):
        self.examples.append((pos, step1, step2))

    def gradient_descent_step(self, epsilon):
        gradient = self._calculate_gradient()
        self._apply_gradient(gradient, epsilon)

    # Function to minimize
    def calc_loss(self):
        total = 0
        for pos, s1, s2 in self.examples:
            total += vec_len(self.device.calculate_cords(s1, s2) - pos)
        return total / len(self.examples)

    def _apply_gradient(self, gradient, epsilon=0.0001):
        g_x1, g_y1, g_x2, g_y2, g_phi1, g_phi2, g_gap = gradient

        # Make a step in the direction of the gradient
        self.device.motor1.pos[0] -= g_x1 * epsilon
        self.device.motor1.pos[1] -= g_y1 * epsilon
        self.device.motor2.pos[0] -= g_x2 * epsilon
        self.device.motor2.pos[1] -= g_y2 * epsilon
        self.device.motor1.align -= g_phi1 * epsilon
        self.device.motor2.align -= g_phi2 * epsilon
        self.device.gap -= g_gap * epsilon

    def _calculate_gradient(self, dx=.0001, dphi=.0001, dgap=.00001):

        # Baseline loss
        loss_0 = self.calc_loss()

        # Calculate the change in Loss for each property numerically
        self.device.motor1.pos[0] += dx
        dL_x1 = self.calc_loss() - loss_0
        self.device.motor1.pos[0] -= dx

        self.device.motor1.pos[1] += dx
        dL_y1 = self.calc_loss() - loss_0
        self.device.motor1.pos[1] -= dx

        self.device.motor2.pos[0] += dx
        dL_x2 = self.calc_loss() - loss_0
        self.device.motor2.pos[0] -= dx

        self.device.motor2.pos[1] += dx
        dL_y2 = self.calc_loss() - loss_0
        self.device.motor2.pos[1] -= dx

        self.device.motor1.align += dphi
        dL_phi1 = self.calc_loss() - loss_0
        self.device.motor1.align -= dphi

        self.device.motor2.align += dphi
        dL_phi2 = self.calc_loss() - loss_0
        self.device.motor2.align -= dphi

        self.device.gap += dgap
        dL_gap = self.calc_loss() - loss_0
        self.device.gap -= dgap

        return [dL_x1 / dx, dL_y1 / dx, dL_x2 / dx, dL_y2 / dx,
                dL_phi1 / dphi, dL_phi2 / dphi, dL_gap / dgap]

    def get_best_guess(self):
        return self.device
