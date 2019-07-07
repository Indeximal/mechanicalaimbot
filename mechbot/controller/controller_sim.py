from collections import deque

import numpy as np
import pygame

from mechbot.utils.vector_utils import vec_len
from mechbot.utils import pygame_utils
from mechbot.controller.simulator import MechanicalSimulator
from mechbot.controller.device import MechanicalDevice, StepperMotor
from mechbot.controller.calibration import CalibrationHelper


# Cordinate conversion
camera = pygame_utils.Camera(150, 640, 320)

# PYGAME INIT
pygame.init()

size = (1280, 720)
cord_size = np.array(size)
screen = pygame.display.set_mode(size)

pygame.display.set_caption("Controller Sim")

clock = pygame.time.Clock()

debug = pygame_utils.LineWriter(10, 10)

# SIMULATION INIT
motor1 = StepperMotor((2., 2.), 200, 1, - 3 / 4 * np.pi)
motor2 = StepperMotor((-2., 2.), 200, 1, - np.pi / 4)

simulation = MechanicalSimulator(motor1, motor2, gap=.2, stick_r=.1, stick_force=.1)

mech_controller = MechanicalDevice(motor1, motor2, gap=.2, stick=.1)
target = (0, 0)

calibrator = CalibrationHelper(simulation.get_interface())

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
            target = cam.world_pos(pygame.mouse.get_pos())
            a, b = mech_controller.calculate_steps(target)
            simulation.cmd_goto(a, b)

    # CALCULATIONS
    simulation.tick(0.01)
    tick_counter += 1

    path.append(camera.pixel(simulation.get_input()))

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

    calibrator.tick(tick_counter)

    # DRAWING
    screen.fill((255, 255, 255))

    if len(path) > 1:
        pygame.draw.lines(screen, (50, 50, 50), False, path)

    for pos, t1, t2 in torque_points:
        c = (int(abs(t1 * 50)), 0, int(abs(t2 * 50)))
        r = int(max(abs(t1), abs(t2)) * 3) + 2
        pygame.draw.circle(screen, c, camera.pixel(pos), r)

    simulation.draw(screen, camera)
    pygame.draw.circle(screen, (110, 110, 200), camera.pixel(target), camera.pixel_len(.1), 2)

    debug.draw(screen)

    pygame.display.flip()
    clock.tick(250)

pygame.quit()