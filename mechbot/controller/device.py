import numpy as np
import pygame

import mechbot.utils.vector_utils as vec


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

    def calculate_angle(self, step):
        return self.align + 2 * np.pi * step / self.steps

    def get_angle(self):
        return self.calculate_angle(self.step)

    def draw(self, screen, cam, r=30):
        pygame.draw.circle(screen, (50, 50, 50), cam.pixel(self.pos), r)
        flag_pos = self.pos + vec.dir_vec(self.get_angle()) * .2
        pygame.draw.line(screen, (200, 200, 200), cam.pixel(
            self.pos), cam.pixel(flag_pos), 3)

    def draw_lines(self, screen, cam, gap):
        normal1 = vec.dir_vec(self.get_angle() + np.pi / 2) * gap
        dir1 = vec.dir_vec(self.get_angle())
        line1r1 = self.pos + normal1
        line1r2 = line1r1 + dir1 * vec.vec_len(self.pos) * 2
        line1l1 = self.pos - normal1
        line1l2 = line1l1 + dir1 * vec.vec_len(self.pos) * 2
        pygame.draw.line(screen, (0, 0, 0), cam.pixel(
            line1r1), cam.pixel(line1r2), 2)
        pygame.draw.line(screen, (0, 0, 0), cam.pixel(
            line1l1), cam.pixel(line1l2), 2)


class VirtualDevice:
    def __init__(self, motor1, motor2, gap, stick):
        self.motor1 = motor1
        self.motor2 = motor2
        self.gap = gap
        self.stick = stick

    def _calculate_step(self, motor, pos):
        # See en.wikipedia.org/wiki/Tangent_lines_to_circles#Outer_tangent
        diff = pos - motor.pos
        motor_angle = np.arctan2(-motor.pos[1], -motor.pos[0])
        direct_angle = np.arctan2(diff[1], diff[0])
        inner_angle = np.arcsin((self.gap - self.stick) / vec.vec_len(diff))
        # Depending on where the stick is one or the other edge should be used
        phi = (direct_angle - inner_angle
               if direct_angle < motor_angle else direct_angle + inner_angle)
        step = (phi - motor.align) * motor.steps / (np.pi * 2)
        return int(step)

    def calculate_steps(self, cord):
        pos = np.array(cord)
        s1 = self._calculate_step(self.motor1, pos)
        s2 = self._calculate_step(self.motor2, pos)
        return s1, s2

    def _get_edges(self, motor, step):
        angle = motor.calculate_angle(step)
        normal_dir = vec.dir_vec(angle + np.pi / 2)
        dead_dist = self.gap - self.stick
        edge1 = (dead_dist * normal_dir + motor.pos, vec.dir_vec(angle))
        edge2 = (-dead_dist * normal_dir + motor.pos, vec.dir_vec(angle))
        return edge1, edge2

    def _calculate_edge(self, motor, step, pos):
        """returns the contraining edge as a ray (origin, direction)
        and whether it is beeing touched"""
        angle = motor.calculate_angle(step)
        normal_dir = vec.dir_vec(angle + np.pi / 2)
        # distance from middle line, used for gap calculation
        dist = np.dot(normal_dir, pos - motor.pos)
        dead_dist = self.gap - self.stick
        if dist > 0:  # Touches edge
            line = (dead_dist * normal_dir + motor.pos, vec.dir_vec(angle))
            return line, dist > dead_dist
        else:  # Toches other edge
            line = (-dead_dist * normal_dir + motor.pos, vec.dir_vec(angle))
            return line, dist < -dead_dist

    def _floating_pos(self, line, other_motor, other_step):
        origin_ray = (np.zeros(2), vec.perp_vec2d(line[1]))
        floating_pos = vec.ray_intersect2d(line, origin_ray)
        _, touch = self._calculate_edge(other_motor, other_step, floating_pos)
        return floating_pos, not touch

    def calculate_cords(self, step1, step2):
        line1, touch1 = self._calculate_edge(self.motor1, step1, np.zeros(2))
        line2, touch2 = self._calculate_edge(self.motor2, step2, np.zeros(2))

        # Centered
        if not touch1 and not touch2:
            return np.zeros(2)

        # The stick only touches the edge of motor 1
        floating_pos, floating = self._floating_pos(line1, self.motor2, step2)
        if floating:
            return floating_pos

        # The stick only touches a edge of motor 2
        floating_pos, floating = self._floating_pos(line2, self.motor1, step1)
        if floating:
            return floating_pos

        # If it doesn't float, return the corner nearest to the center
        edge1r, edge1l = self._get_edges(self.motor1, step1)
        edge2r, edge2l = self._get_edges(self.motor2, step2)
        corners = [vec.ray_intersect2d(r1, r2) for r1, r2 in [(edge1r, edge2r),
            (edge1r, edge2l), (edge1l, edge2r), (edge1l, edge2l)]]
        corners.sort(key=lambda p: vec.vec_len(p))
        return corners[0]

    def draw(self, screen, cam):
        pygame.draw.circle(screen, (210, 210, 210),
                           cam.pixel((0, 0)), cam.pixel_len(1), 3)

        self.motor1.draw(screen, cam)
        self.motor1.draw_lines(screen, cam, self.gap)

        self.motor2.draw(screen, cam)
        self.motor2.draw_lines(screen, cam, self.gap)

