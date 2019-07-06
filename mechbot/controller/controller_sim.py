from collections import deque

import numpy as np
import pygame


def direction_vec(angle):
    return np.array([np.cos(angle), np.sin(angle)])


def vec_len(vec):
    return np.linalg.norm(vec)


class LineWriter:
    def __init__(self, x, y, h=30, size=24, color=(0, 0, 0)):
        self.font = pygame.font.Font(None, size)
        self.x = x
        self.y = y
        self.h = h
        self.color = color
        self.lines = []

    def print(self, text):
        self.lines.append(str(text))

    def draw(self, screen):
        y = self.y
        for line in self.lines:
            text_surface = self.font.render(line, True, self.color)
            screen.blit(text_surface, (self.x, y))
            y += self.h
        self.lines = []


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

    def draw(self, screen, r=30):
        pygame.draw.circle(screen, (50, 50, 50), pixel(self.pos), r)
        flag_pos = self.pos + direction_vec(self.get_angle()) * .2
        pygame.draw.line(screen, (200, 200, 200), pixel(self.pos), pixel(flag_pos), 3)

    def draw_lines(self, screen, gap):
        normal1 = direction_vec(self.get_angle() + np.pi / 2) * gap
        dir1 = direction_vec(self.get_angle())
        line1r1 = self.pos + normal1
        line1r2 = line1r1 + dir1 * vec_len(self.pos) * 2
        line1l1 = self.pos - normal1
        line1l2 = line1l1 + dir1 * vec_len(self.pos) * 2
        pygame.draw.line(screen, (0, 0, 0), pixel(line1r1), pixel(line1r2), 2)
        pygame.draw.line(screen, (0, 0, 0), pixel(line1l1), pixel(line1l2), 2)


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

    def get_input(self):
        return self.stick_pos

    def cmd_goto(self, a, b):
        self.move = True
        self.target1 = a
        self.target2 = b

    def torque_on_motor(self, motor):
        normal_dir = direction_vec(motor.get_angle() + np.pi / 2)
        normal_force_mag = np.dot(self.center_force, normal_dir)
        distance = vec_len(self.stick_pos - motor.pos)
        return distance * normal_force_mag

    def _calculate_force(self, motor):
        normal_dir = direction_vec(motor.get_angle() + np.pi / 2)
        normal_force_mag = np.dot(self.center_force, normal_dir)
        # distance from middle line, used for gap calculation
        dist = np.dot(normal_dir, self.stick_pos - motor.pos)
        dead_dist = self.gap - self.stick_r
        force = 0
        if dist > dead_dist: # Touches edge
            # Normal force plus some correction for motion
            force = normal_force_mag + 6 * (dist + dead_dist)
        elif dist < - dead_dist: # Toches other edge
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
        self.force_vec_1 = self._calculate_force(motor1)
        self.force_vec_2 = self._calculate_force(motor2)

        self.stick_pos += dt * (self.force_vec_1 + self.force_vec_2 + self.center_force)
        # self.stick_pos += dt * self.stick_vel
        # self.stick_vel *= .95

    def draw(self, screen):
        motor2.draw(screen)
        pygame.draw.circle(screen, (210, 210, 210), pixel((0, 0)), pixel_len(1), 3)
        pygame.draw.circle(screen, (110, 110, 110), pixel(self.stick_pos), pixel_len(self.stick_r))

        motor1.draw(screen)
        motor1.draw_lines(screen, self.gap)

        motor2.draw(screen)
        motor2.draw_lines(screen, self.gap)

        pygame.draw.line(screen, (255, 0, 0), pixel(self.stick_pos), pixel(self.stick_pos + self.force_vec_1 * .2), 3)
        pygame.draw.line(screen, (0, 255, 0), pixel(self.stick_pos), pixel(self.stick_pos + self.force_vec_2 * .2), 3)
        pygame.draw.line(screen, (0, 0, 255), pixel(self.stick_pos), pixel(self.stick_pos + self.center_force * .2), 3)


class MechanicalController:
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


# Cordinate conversion
pixel_scale = 150
pixel_shift = (640, 320)

def pixel_len(l):
    return int(l * pixel_scale)

def pixel(pos):
    x, y = pos
    dx, dy = pixel_shift
    return (int(x * pixel_scale + dx), int(y * pixel_scale + dy))

def world_pos(pixel):
    x, y = pixel
    dx, dy = pixel_shift
    return ((x - dx) / pixel_scale, (y - dy) / pixel_scale)


# PYGAME INIT
pygame.init()
pygame.font.init()

size = (1280, 720)
cord_size = np.array(size)
screen = pygame.display.set_mode(size)

pygame.display.set_caption("Controller Sim")

clock = pygame.time.Clock()

debug = LineWriter(10, 10)

# SIMULATION INIT
motor1 = StepperMotor((2., 2.), 200, 1, - 3 / 4 * np.pi)
motor2 = StepperMotor((-2., 2.), 200, 1, - np.pi / 4)

simulation = MechanicalSimulator(motor1, motor2, gap=.2, stick_r=.1, stick_force=.1)

mech_controller = MechanicalController(motor1, motor2, gap=.2, stick=.1)
target = (0, 0)

tick_counter = 0

path = deque(maxlen=250)
force_1_queue = deque(maxlen=30)
force_2_queue = deque(maxlen=30)
torque_points = []

def record_torque():
    t1 = sum(force_1_queue) / len(force_1_queue)
    t2 = sum(force_2_queue) / len(force_2_queue)
    torque_points.append((tuple(simulation.stick_pos), t1, t2))

collecting = False
step_range = 7
step1 = -step_range
step2 = -step_range

#  MAIN LOOP
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                motor1.left()
            if event.key == pygame.K_RIGHT:
                motor1.right()
            if event.key == pygame.K_UP:
                motor2.left()
            if event.key == pygame.K_DOWN:
                motor2.right()
            if event.key == pygame.K_p:
                record_torque()
            if event.key == pygame.K_o:
                print(torque_points)
            if event.key == pygame.K_ESCAPE:
                running = False
        # Move target
        if event.type == pygame.MOUSEBUTTONUP:
            target = world_pos(pygame.mouse.get_pos())
            a, b = mech_controller.calculate_steps(target)
            simulation.cmd_goto(a, b)

    # CALCULATIONS
    simulation.tick(0.01)
    tick_counter += 1

    path.append(pixel(simulation.get_input()))

    force_1_queue.append(simulation.torque_on_motor(motor1))
    force_2_queue.append(simulation.torque_on_motor(motor2))
    debug.print("1: {:.2f}".format(sum(force_1_queue) / len(force_1_queue)))
    debug.print("2: {:.2f}".format(sum(force_2_queue) / len(force_2_queue)))

    if tick_counter % 5 == 0:
        simulation.loop()

    if tick_counter % 300 == 0 and collecting:
        record_torque()
        simulation.cmd_goto(step1*2, step2*2)
        step1 += 1
        if step1 > step_range:
            step1 = -step_range
            step2 += 1
            if step2 > step_range:
                collecting = False

    # DRAWING
    screen.fill((255, 255, 255))

    if len(path) > 1:
        pygame.draw.lines(screen, (50, 50, 50), False, path)

    for pos, t1, t2 in torque_points:
        c = (int(abs(t1 * 50)), 0, int(abs(t2 * 50)))
        r = int(max(abs(t1), abs(t2)) * 3) + 2
        pygame.draw.circle(screen, c, pixel(pos), r)

    simulation.draw(screen)
    pygame.draw.circle(screen, (110, 110, 200), pixel(target), pixel_len(.1), 2)

    debug.draw(screen)

    pygame.display.flip()
    clock.tick(250)

pygame.quit()