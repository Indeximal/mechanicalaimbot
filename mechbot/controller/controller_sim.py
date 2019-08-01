from collections import deque

import numpy as np
import pygame

from mechbot.utils.vector_utils import vec_len
from mechbot.utils import pygame_utils
from mechbot.controller.simulator import MechanicalSimulator
from mechbot.controller.device import VirtualDevice, StepperMotor
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
motor1 = StepperMotor((1.6, -1.4), 200, 1, 2.44)
motor2 = StepperMotor((-1.7, -1.5), 200, 1, 0.76)
# motor1.align = np.arctan2(-motor1.pos[1], -motor1.pos[0])
# motor2.align = np.arctan2(-motor2.pos[1], -motor2.pos[0])
mech_device = VirtualDevice(motor1, motor2, gap=.2, stick=.1)

simulation = MechanicalSimulator(mech_device, stick_force=.099)

target = (0, 0)

calibrator = CalibrationHelper(simulation.get_interface(), motor1.steps)

tick_counter = 0

path = deque(maxlen=250)
force_1_queue = deque(maxlen=30)
force_2_queue = deque(maxlen=30)

show_guess = False


#  MAIN LOOP
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                motor1.right()
                simulation.move = False
            if event.key == pygame.K_RIGHT:
                motor1.left()
                simulation.move = False
            if event.key == pygame.K_UP:
                motor2.right()
                simulation.move = False
            if event.key == pygame.K_DOWN:
                motor2.left()
                simulation.move = False
            if event.key == pygame.K_p:
                guess = calibrator.get_result()
                print("[{m1.pos[0]:.3f}|{m1.pos[1]:.3f}, {m2.pos[0]:.3f}|"
                      "{m2.pos[1]:.3f}, {m1.align:.3f}, {m2.align:.3f}, {gap:.4f}]"
                      .format(m1=guess.motor1, m2=guess.motor2,
                              gap=guess.gap))
                show_guess = True
            if event.key == pygame.K_c:
                calibrator.start()
            if event.key == pygame.K_ESCAPE:
                running = False
        # Move target
        if event.type == pygame.MOUSEBUTTONUP:
            target = camera.world_pos(pygame.mouse.get_pos())
            a, b = mech_device.calculate_steps(target)
            simulation.cmd_goto(a, b)

    # CALCULATIONS
    simulation.tick(0.01)
    calibrator.tick()
    if tick_counter % 5 == 0:
        simulation.loop()

    tick_counter += 1

    path.append(camera.pixel(simulation.get_input()))

    force_1_queue.append(simulation.torque_on_motor(motor1))
    force_2_queue.append(simulation.torque_on_motor(motor2))
    debug.print("1: {:.2f}".format(sum(force_1_queue) / len(force_1_queue)))
    debug.print("2: {:.2f}".format(sum(force_2_queue) / len(force_2_queue)))

    calculated_pos = mech_device.calculate_cords(motor1.step, motor2.step)

    # DRAWING
    # clear
    screen.fill((255, 255, 255))

    # path
    if len(path) > 1:
        pygame.draw.lines(screen, (50, 50, 50), False, path)

    for pos, _, _ in calibrator.m1_points:
        pygame.draw.circle(screen, (0, 0, 0), camera.pixel(pos), 4, 2)
    for pos, _, _ in calibrator.m2_points:
        pygame.draw.circle(screen, (0, 0, 0), camera.pixel(pos), 4, 2)
    for pos, _, _ in calibrator.circle_points:
        pygame.draw.circle(screen, (0, 0, 0), camera.pixel(pos), 4, 2)

    # Mechanics
    simulation.draw(screen, camera)
    if show_guess:
        calibrator.get_result().draw(screen, camera)

    # Target and guess
    pygame.draw.circle(screen, (110, 110, 200), camera.pixel(
        target), camera.pixel_len(.1), 2)
    pygame.draw.circle(screen, (200, 110, 110), camera.pixel(
        calculated_pos), camera.pixel_len(.11), 2)

    debug.print(clock.tick(60))

    # Text
    debug.draw(screen)

    pygame.display.flip()

pygame.quit()
