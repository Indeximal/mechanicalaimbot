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
motor1 = StepperMotor((2., 2.), 200, 1, - 3 / 4 * np.pi)
motor2 = StepperMotor((-2., 2.), 200, 1, - np.pi / 4)

simulation = MechanicalSimulator(
    motor1, motor2, gap=.2, stick_r=.1, stick_force=.1)

mech_controller = VirtualDevice(motor1, motor2, gap=.2, stick=.1)
target = (0, 0)

# initial_motor_1 = StepperMotor((1.8, 2.1), 200, 1, -2.51)
# initial_motor_2 = StepperMotor((-2.2, 1.94), 200, 1, -0.65)
# initial_device = VirtualDevice(initial_motor_1, initial_motor_2, .14, 0.)

calibrator = CalibrationHelper(simulation.get_interface())

# calibration_points = []

tick_counter = 0

path = deque(maxlen=250)
force_1_queue = deque(maxlen=30)
force_2_queue = deque(maxlen=30)


# def record_calibration_point():
#     point = (simulation.get_input(), motor1.step, motor2.step)
#     calibration_points.append(point)
#     calibrator.add_example(motor1.step, motor2.step, simulation.get_input())


# def calibrate():
#     epsilon = 0.1
#     for epoch in range(50):
#         calibrator.gradient_descent_step(epsilon)
#         loss = calibrator.calc_loss()
#         if loss < .1:
#             epsilon = 0.01
#         print(epoch, loss, initial_motor_1.pos, initial_motor_2.pos)


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
            # if event.key == pygame.K_p:
            #     record_calibration_point()
            if event.key == pygame.K_c:
                calibrator.start()
            # if event.key == pygame.K_o:
            #     print(calibration_points)
            if event.key == pygame.K_ESCAPE:
                running = False
        # Move target
        if event.type == pygame.MOUSEBUTTONUP:
            target = camera.world_pos(pygame.mouse.get_pos())
            a, b = mech_controller.calculate_steps(target)
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

    calculated_pos = mech_controller.calculate_cords(motor1.step, motor2.step)

    # DRAWING
    # clear
    screen.fill((255, 255, 255))

    # path
    if len(path) > 1:
        pygame.draw.lines(screen, (50, 50, 50), False, path)

    for _, pos in calibrator.m1_points:
        pygame.draw.circle(screen, (0, 0, 0), camera.pixel(pos), 4, 2)

    # Mechanics
    simulation.draw(screen, camera)

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
