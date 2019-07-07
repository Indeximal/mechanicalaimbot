import numpy as np
import pygame

from mechbot.utils.vector_utils import vec_len, dir_vec


class MechanicalSimulator:
    def __init__(self, motor1, motor2, gap, stick_r, stick_force):
        self.motor1 = motor1
        self.motor2 = motor2
        self.gap = gap
        self.stick_r = stick_r
        self.stick_force = stick_force
        self.stick_pos = np.array([0., 0.])
        # self.stick_vel = np.array([0., 0.])
        self.target1 = 0
        self.target2 = 0
        self.move = False

    def get_interface(self):
        return self

    def get_input(self):
        return np.array(self.stick_pos)

    def cmd_goto(self, a, b):
        self.move = True
        self.target1 = a
        self.target2 = b

    def torque_on_motor(self, motor):
        normal_dir = dir_vec(motor.get_angle() + np.pi / 2)
        normal_force_mag = np.dot(self.center_force, normal_dir)
        distance = vec_len(self.stick_pos - motor.pos)
        return distance * normal_force_mag

    def _calculate_force(self, motor):
        normal_dir = dir_vec(motor.get_angle() + np.pi / 2)
        normal_force_mag = np.dot(self.center_force, normal_dir)
        # distance from middle line, used for gap calculation
        dist = np.dot(normal_dir, self.stick_pos - motor.pos)
        dead_dist = self.gap - self.stick_r
        force = 0
        if dist > dead_dist:  # Touches edge
            # Normal force plus some correction for motion
            force = normal_force_mag + 6 * (dist + dead_dist)
        elif dist < - dead_dist:  # Toches other edge
            force = normal_force_mag + 6 * (dist - dead_dist)
        return (-normal_dir * force)

    def loop(self):
        # Step motors, normally done on arduino
        if self.move:
            if self.motor1.step < self.target1:
                self.motor1.left()
            elif self.motor1.step > self.target1:
                self.motor1.right()
            if self.motor2.step < self.target2:
                self.motor2.left()
            elif self.motor2.step > self.target2:
                self.motor2.right()

    def tick(self, dt):
        # Update stick position
        self.center_force = np.array([0., 0.])
        if vec_len(self.stick_pos) > .02:
            self.center_force = - 1 * self.stick_pos / vec_len(self.stick_pos)

        # Calculate normal force on the stick
        # TODO: calcucalte physical normal force
        self.force_vec_1 = self._calculate_force(self.motor1)
        self.force_vec_2 = self._calculate_force(self.motor2)

        self.stick_pos += dt * (self.force_vec_1 +
                                self.force_vec_2 + self.center_force)
        # self.stick_pos += dt * self.stick_vel
        # self.stick_vel *= .95

    def draw(self, screen, cam):
        pygame.draw.circle(screen, (210, 210, 210),
                           cam.pixel((0, 0)), cam.pixel_len(1), 3)
        pygame.draw.circle(screen, (110, 110, 110), cam.pixel(
            self.stick_pos), cam.pixel_len(self.stick_r))

        self.motor1.draw(screen, cam)
        self.motor1.draw_lines(screen, cam, self.gap)

        self.motor2.draw(screen, cam)
        self.motor2.draw_lines(screen, cam, self.gap)

        pygame.draw.line(screen, (255, 0, 0), cam.pixel(
            self.stick_pos), cam.pixel(self.stick_pos + self.force_vec_1 * .2), 3)
        pygame.draw.line(screen, (0, 255, 0), cam.pixel(
            self.stick_pos), cam.pixel(self.stick_pos + self.force_vec_2 * .2), 3)
        pygame.draw.line(screen, (0, 0, 255), cam.pixel(
            self.stick_pos), cam.pixel(self.stick_pos + self.center_force * .2), 3)
