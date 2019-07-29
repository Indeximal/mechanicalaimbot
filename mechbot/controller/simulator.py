import numpy as np
import pygame

from mechbot.utils.vector_utils import vec_len, dir_vec


class MechanicalSimulator:
    def __init__(self, device, stick_force):
        self.device = device
        self.stick_force = stick_force
        self.stick_pos = np.array([0., 0.])
        self.target1 = 0
        self.target2 = 0
        self.move = False

    def get_interface(self):
        """returns an object with the methods get_input and cmd_goto"""
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
        dead_dist = self.device.gap - self.device.stick
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
            if self.device.motor1.step < self.target1:
                self.device.motor1.left()
            elif self.device.motor1.step > self.target1:
                self.device.motor1.right()
            if self.device.motor2.step < self.target2:
                self.device.motor2.left()
            elif self.device.motor2.step > self.target2:
                self.device.motor2.right()

    def tick(self, dt):
        # Update stick position
        self.center_force = np.array([0., 0.])
        if vec_len(self.stick_pos) > .02:
            self.center_force = - 1 * self.stick_pos / vec_len(self.stick_pos)

        # Calculate normal force on the stick
        self.force_vec_1 = self._calculate_force(self.device.motor1)
        self.force_vec_2 = self._calculate_force(self.device.motor2)

        self.stick_pos += dt * (self.force_vec_1 +
                                self.force_vec_2 + self.center_force)

    def draw(self, screen, cam):
        # Draw motors
        self.device.draw(screen, cam)

        # Draw stick
        pygame.draw.circle(screen, (110, 110, 110), cam.pixel(
            self.stick_pos), cam.pixel_len(self.device.stick))

        # Draw forces
        pygame.draw.line(screen, (255, 0, 0), cam.pixel(
            self.stick_pos), cam.pixel(self.stick_pos + self.force_vec_1 * .2), 3)
        pygame.draw.line(screen, (0, 255, 0), cam.pixel(
            self.stick_pos), cam.pixel(self.stick_pos + self.force_vec_2 * .2), 3)
        pygame.draw.line(screen, (0, 0, 255), cam.pixel(
            self.stick_pos), cam.pixel(self.stick_pos + self.center_force * .2), 3)
