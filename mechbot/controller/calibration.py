import logging
import time
from collections import deque

import numpy as np

from mechbot.utils.vector_utils import vec_len, dir_vec
from mechbot.controller.device import StepperMotor, VirtualDevice


class NotReadyError(Exception):
    pass


class LogicError(Exception):
    def __init__(self, message):
        Exception.__init__(self, message)


class MovementStateEnum:
    Idle = 1
    Waiting = 2
    Moving = 3


class MoveController:
    def __init__(self, interface, motion_threshold=.03, wait_ticks=20,
                 wait_duration=.4):
        # Settings
        self.motion_threshold = motion_threshold  # units
        self.motion_path = deque(maxlen=wait_ticks)  # ticks
        self.wait_duration = wait_duration  # seconds

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
    Wait = 0
    M1_Prepare = 1
    M1_Gather = 2
    M2_Prepare = 3
    M2_Gather = 4
    Gather_Circle = 5
    Done = 7


class CalibrationHelper(MoveController):
    """This class takes control of an interface to approximate the device"""

    def __init__(self, interface, motor_steps, max_deflection=.3,
                 center_threshold=.05, circle_1_radius=.8, circle_2_radius=.5,
                 angular_step_1=.4, angular_step_2=.8, do_optimization=True,
                 motion_threshold=.03, wait_ticks=20, wait_duration=.4):
        # Init
        MoveController.__init__(self, interface, wait_ticks=wait_ticks,
            motion_threshold=motion_threshold, wait_duration=wait_duration)
        self.m1_points = []
        self.m2_points = []
        self.circle_points = []
        self.state = CalibrationStateEnum.Wait
        self.motor_steps = motor_steps
        # Settings
        self.do_optimization = do_optimization
        self.max_deflection = max_deflection
        self.center_threshold = center_threshold
        self.circle_1_radius = circle_1_radius
        self.circle_2_radius = circle_2_radius
        self.angular_step_1 = angular_step_1
        self.angular_step_2 = angular_step_2

    def compute_guess(self):
        motors = []
        gap = 0.

        dphi = 2 * np.pi / self.motor_steps

        for data in [self.m1_points, self.m2_points]:
            points = np.array([p for p, _, _ in data])
            xs = points[..., 0]
            ys = points[..., 1]
            line = np.polyfit(xs, ys, 1)
            angle = np.arctan(line[0]) + np.pi / 2

            direction = dir_vec(angle)
            orientation_first = np.sign(np.dot(direction, points[0]))
            orientation_last = np.sign(np.dot(direction, points[-1]))
            if orientation_first != orientation_last:
                raise LogicError("The first and last point contradict!")
            # Flip the motor according to curvature of the points
            if orientation_first == 1:
                angle -= np.pi
                direction *= -1

            perpendicular_vec = dir_vec(angle + np.pi / 2)

            # Compute the avg distances between two non-center-steps
            non_zero_data = [(p, s1, s2) for p, s1, s2 in data if vec_len(
                p) > self.center_threshold]
            distances = []
            point_gap = 0
            for vec in [perpendicular_vec, -perpendicular_vec]:
                ps = [p for p, _, _ in non_zero_data if np.dot(p, vec) < 0]
                if len(ps) < 2:
                    raise LogicError("Not enough points away from the center!")
                distances += [vec_len(p - q) for p, q in zip(ps[1:], ps[:-1])]
                smallest_dist_to_center = min(ps, key=lambda p: vec_len(p))
                point_gap += vec_len(smallest_dist_to_center)
            dist = sum(distances) / len(distances)

            # Use that to compute the radius away from the motor
            radius = dist / (2 * np.sin(dphi / 2))
            position = - direction * radius

            theorectial_gap = (len(points) - len(non_zero_data) + 1) * dist

            # Full gap is the missing distance, but convention: half gap
            gap += (theorectial_gap - point_gap) / 2

            # Gap angle if the edge was a V-shape.
            beta = np.arcsin(gap / radius)
            aligns = []
            for sign in [1, -1]:
                vec = sign * perpendicular_vec
                ps = [(p, s1 if len(motors) == 0 else s2) for p, s1, s2 in
                      non_zero_data if np.dot(p, vec) < 0]

                mx, my = position
                angles = [(np.arctan2(y - my, x - mx), s) for (x, y), s in ps]
                aligns += [phi + sign * beta - s * dphi for phi, s in angles]

            align = sum(aligns) / len(aligns)

            motor = StepperMotor(position, self.motor_steps, 1, align)
            motors.append(motor)

        gap /= 2

        return VirtualDevice(motors[0], motors[1], gap + .1, .1)

    def optimize_guess(self, iterations, epsilon):
        all_examples = self.m1_points + self.m2_points + self.circle_points
        helper = GradientDescentHelper(self.device_guess, all_examples)

        for epoch in range(iterations):
            helper.gradient_descent_step(epsilon)

    def calculate_loss(self):
        all_examples = self.m1_points + self.m2_points + self.circle_points
        helper = GradientDescentHelper(self.device_guess, all_examples)

        return helper.calc_loss()

    def _act(self):
        # Function for evaluation
        def reaction(steps, pos, t):
            over_deflection = vec_len(pos) > self.max_deflection

            if self.state == CalibrationStateEnum.M1_Prepare:
                # Extended enough
                if over_deflection:
                    self.state = CalibrationStateEnum.M1_Gather

            if self.state == CalibrationStateEnum.M1_Gather:
                # Gather floating points initial guess
                self.m1_points.append((pos, steps[0], steps[1]))
                if self.step_1 < 0 and over_deflection:
                    self.state = CalibrationStateEnum.M2_Prepare

            if self.state == CalibrationStateEnum.M2_Prepare:
                if self.step_2 > 0 and over_deflection:
                    self.state = CalibrationStateEnum.M2_Gather

            if self.state == CalibrationStateEnum.M2_Gather:
                # Gather floating points initial guess
                self.m2_points.append((pos, steps[0], steps[1]))
                if self.step_2 < 0 and over_deflection:
                    # Compute a initial VirtualDevice guess
                    self.device_guess = self.compute_guess()
                    if self.do_optimization:
                        self.state = CalibrationStateEnum.Gather_Circle
                    else:
                        self.state = CalibrationStateEnum.Done

            if self.state == CalibrationStateEnum.Gather_Circle:
                # Gather points on a cirlce for gradient descent
                self.circle_points.append((pos, steps[0], steps[1]))
                if self.angle > 2 * np.pi:
                    self.circle_radius = self.circle_2_radius
                    self.angular_step = self.angular_step_2
                if self.angle > 4 * np.pi:
                    # Finish by doing some optimization
                    self.optimize_guess(10, 0.001)
                    self.state = CalibrationStateEnum.Done

            self._act()

        # Movements
        if self.state == CalibrationStateEnum.M1_Prepare:
            self.step_1 += 1
            self.move_to(self.step_1, 0, reaction)

        if self.state == CalibrationStateEnum.M1_Gather:
            self.step_1 -= 1
            self.move_to(self.step_1, 0, reaction)

        if self.state == CalibrationStateEnum.M2_Prepare:
            self.step_2 += 1
            self.move_to(0, self.step_2, reaction)

        if self.state == CalibrationStateEnum.M2_Gather:
            self.step_2 -= 1
            self.move_to(0, self.step_2, reaction)

        if self.state == CalibrationStateEnum.Gather_Circle:
            # Move to a points on a circle
            position = dir_vec(self.angle) * self.circle_radius
            s1, s2 = self.device_guess.calculate_steps(position)
            self.move_to(s1, s2, reaction)
            self.angle += self.angular_step

    def start(self):
        if self.state != CalibrationStateEnum.Wait:
            raise LogicError("This class can only be started once!")
        self.state = CalibrationStateEnum.M1_Prepare
        self.step_1 = 0
        self.step_2 = 0
        self.angle = 0
        self.circle_radius = self.circle_1_radius
        self.angular_step = self.angular_step_1
        self._act()

    def is_done(self):
        return self.state == CalibrationStateEnum.Done

    def get_result(self):
        return self.device_guess


class AlignmentHelper(MoveController):
    """This class takes control of an interface find the motor alignment"""

    def __init__(self, interface, center_threshold=.05, wait_ticks=20,
                 max_deflection=.3, motion_threshold=.03, wait_duration=.4):
        # Init
        MoveController.__init__(self, interface, wait_ticks=wait_ticks,
                                motion_threshold=motion_threshold,
                                wait_duration=wait_duration)
        self.step_2 = 0
        self.step_1 = 0
        self.m1_points = []
        self.m2_points = []
        self.state = CalibrationStateEnum.Wait
        self.center_threshold = center_threshold
        self.max_deflection = max_deflection

    def apply_to_device(self, device):
        for nr, data, motor in [(1, self.m1_points, device.motor1),
                                (2, self.m2_points, device.motor2)]:
            # Information given by the config
            dphi = 2 * np.pi / motor.steps
            mx, my = motor.pos
            radius = vec_len(motor.pos)

            # Data points outside the center
            non_zero_data = [(p, s1, s2) for p, s1, s2 in data if vec_len(
                p) > self.center_threshold]

            angle = np.arctan2(-my, -mx)
            perpendicular_vec = dir_vec(angle + np.pi / 2)

            # Calculate the align of the motors
            # Gap angle if the edge was a V-shape.
            beta = np.arcsin(device.gap / radius)
            aligns = []
            for sign in [1, -1]:
                vec = sign * perpendicular_vec
                ps = [(p, s1 if nr == 1 else s2) for p, s1, s2 in
                      non_zero_data if np.dot(p, vec) < 0]

                angles = [(np.arctan2(y - my, x - mx), s) for (x, y), s in ps]
                aligns += [phi + sign * beta - s * dphi for phi, s in angles]

            align = sum(aligns) / len(aligns)
            motor.align = align

    def _act(self):
        # Function for evaluation
        def reaction(steps, pos, t):
            over_deflected = vec_len(pos) > self.max_deflection

            if self.state == CalibrationStateEnum.M1_Prepare:
                # Extended enough
                if over_deflected:
                    self.state = CalibrationStateEnum.M1_Gather

            if self.state == CalibrationStateEnum.M1_Gather:
                # Gather floating points initial guess
                self.m1_points.append((pos, steps[0], steps[1]))
                if self.step_1 < 0 and over_deflected:
                    self.state = CalibrationStateEnum.M2_Prepare

            if self.state == CalibrationStateEnum.M2_Prepare:
                if self.step_2 > 0 and over_deflected:
                    self.state = CalibrationStateEnum.M2_Gather

            if self.state == CalibrationStateEnum.M2_Gather:
                # Gather floating points initial guess
                self.m2_points.append((pos, steps[0], steps[1]))
                if self.step_2 < 0 and over_deflected:
                    # Done, results can now be applied
                    self.state = CalibrationStateEnum.Done

            self._act()

        # Movements
        if self.state == CalibrationStateEnum.M1_Prepare:
            self.step_1 += 1
            self.move_to(self.step_1, 0, reaction)

        if self.state == CalibrationStateEnum.M1_Gather:
            self.step_1 -= 1
            self.move_to(self.step_1, 0, reaction)

        if self.state == CalibrationStateEnum.M2_Prepare:
            self.step_2 += 1
            self.move_to(0, self.step_2, reaction)

        if self.state == CalibrationStateEnum.M2_Gather:
            self.step_2 -= 1
            self.move_to(0, self.step_2, reaction)

    def start(self):
        if self.state != CalibrationStateEnum.Wait:
            raise LogicError("This class can only be started once!")
        self.state = CalibrationStateEnum.M1_Prepare
        self._act()

    def is_done(self):
        return self.state == CalibrationStateEnum.Done


class GradientDescentHelper:
    def __init__(self, device, examples=[]):
        self.device = device
        self.examples = examples

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
