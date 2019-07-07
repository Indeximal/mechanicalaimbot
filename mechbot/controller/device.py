import numpy as np
import pygame

from mechbot.utils.vector_utils import vec_len, direction_vec


class StepperMotor:
    def __init__(self, pos, steps, microsteps, align):
        super(StepperMotor, self).__init__()
        self.steps = steps * microsteps
        self.pos = np.array(pos)
        self.align = align
        self.step = 0

    def left(self):
        self.step += 1

    def right(self):
        self.step -= 1

    def get_angle(self):
        return self.align + 2 * np.pi * self.step / self.steps

    def draw(self, screen, cam, r=30):
        pygame.draw.circle(screen, (50, 50, 50), cam.pixel(self.pos), r)
        flag_pos = self.pos + direction_vec(self.get_angle()) * .2
        pygame.draw.line(screen, (200, 200, 200), cam.pixel(self.pos), cam.pixel(flag_pos), 3)

    def draw_lines(self, screen, cam, gap):
        normal1 = direction_vec(self.get_angle() + np.pi / 2) * gap
        dir1 = direction_vec(self.get_angle())
        line1r1 = self.pos + normal1
        line1r2 = line1r1 + dir1 * vec_len(self.pos) * 2
        line1l1 = self.pos - normal1
        line1l2 = line1l1 + dir1 * vec_len(self.pos) * 2
        pygame.draw.line(screen, (0, 0, 0), cam.pixel(line1r1), cam.pixel(line1r2), 2)
        pygame.draw.line(screen, (0, 0, 0), cam.pixel(line1l1), cam.pixel(line1l2), 2)


class MechanicalDevice:
    def __init__(self, motor1, motor2, gap, stick):
        self.motor1 = motor1
        self.motor2 = motor2
        self.gap = gap
        self.stick = stick

    def _calculate_step(self, motor, pos):
        # See https://en.wikipedia.org/wiki/Tangent_lines_to_circles#Outer_tangent
        diff = pos - motor.pos
        motor_angle = np.arctan2(-motor.pos[1], -motor.pos[0])
        direct_angle = np.arctan2(diff[1], diff[0])
        inner_angle = np.arcsin((self.gap - self.stick) / vec_len(diff))
        # Depending on where the stick is one or the other edge should be used
        phi = (direct_angle - inner_angle) if direct_angle < motor_angle else (direct_angle + inner_angle)
        step = (phi - motor.align) * motor.steps / (np.pi * 2)
        return int(step)

    def calculate_steps(self, cord):
        pos = np.array(cord)
        return self._calculate_step(motor1, pos), self._calculate_step(motor2, pos)

