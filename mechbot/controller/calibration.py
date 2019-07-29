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
        self.motion_threshold = .05
        self.motion_path = deque(maxlen=3)
        self.wait_duration = 1.

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
        self.callback(self.steps, pos, time.time() - self.initial_time)
        self.move_state = MovementStateEnum.Idle

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
    M1_prepare = 1
    M1_gather = 2


class CalibrationHelper(MoveController):
    def __init__(self, interface, wait_duration=70,
                 max_deflection=.2):
        super(CalibrationHelper, self).__init__(interface)

        # self.device = initial_device
        self.step_1 = None
        self.step_2 = None
        self.phase = "m1_pos"
        self.subphase = 0
        self.done = False
        self.next_action = time.time()
        self.m1_points = []
        self.last_pos = interface.get_input()
        self.motion_threshold = .05

        self.wait_duration = wait_duration
        self.max_deflection = max_deflection
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

    def _act_old(self):
        input_pos = self.interface.get_input()
        deflection = vec_len(input_pos)

        if self.phase == "m1_pos":
            if self.step_1 is not None:
                self.m1_points.append((self.step_1, input_pos))
                self.step_1 += 1
            else:
                self.step_1 = 0

            if deflection < self.max_deflection:
                self.interface.cmd_goto(self.step_1, 0)

            else:
                self.interface.cmd_goto(0, 0)
                self.phase == "m1_neg"
        elif self.phase == "m1_neg":
            if self.step_1 is not None:
                self.m1_points.append((self.step_1, input_pos))
                self.step_1 += 1
            else:
                self.step_1 = 0

            if deflection < self.max_deflection:
                self.interface.cmd_goto(self.step_1, 0)
            else:
                self.phase == "m1_neg"
        else:
            self.done = True


    def is_done(self):
        return self.done

    def get_best_guess(self):
        return self.device
