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

initial_motor_1 = StepperMotor((2.8, 2.1), 200, 1, -2.51)
initial_motor_2 = StepperMotor((-2.5, 0.94), 200, 1, -0.65)
initial_device = VirtualDevice(initial_motor_1, initial_motor_2, .14, 0.)

calibrator = CalibrationHelper(simulation.get_interface(), initial_device)

calibration_points = [(np.array([ 0.01166689, -0.00912563]), 1, 0), (np.array([ 0.12092835, -0.10045304]), 3, 0), (np.array([ 0.27449883, -0.20465463]), 5, 0), (np.array([ 0.42277604, -0.2924636 ]), 7, 0), (np.array([ 0.54410514, -0.10222947]), 7, 4), (np.array([0.42123738, 0.00534241]), 5, 4), (np.array([0.29696039, 0.08145666]), 3, 4), (np.array([0.18568755, 0.19216418]), 1, 4), (np.array([0.15589698, 0.1986122 ]), -1, 4), (np.array([0.06525337, 0.26944048]), -3, 4), (np.array([-0.06651816,  0.36750731]), -5, 4), (np.array([-0.19252463,  0.4821675 ]), -7, 4), (np.array([-0.32384476,  0.61937707]), -9, 4), (np.array([-0.21181797,  0.66689663]), -9, 6), (np.array([3.99072985e-04, 7.96086106e-01]), -9, 9), (np.array([0.2016434 , 0.66733924]), -6, 9), (np.array([0.32044651, 0.61680363]), -2, 9), (np.array([0.46947578, 0.41343054]), 2, 8), (np.array([0.59391324, 0.23132401]), 5, 7), (np.array([0.77476793, 0.13195244]), 8, 7), (np.array([ 0.78896271, -0.1475068 ]), 10, 5), (np.array([ 0.66537986, -0.37880229]), 10, 1), (np.array([ 0.38573302, -0.72955928]), 9, -3), (np.array([ 0.20061834, -0.49018643]), 6, -3), (np.array([ 0.07290623, -0.33711895]), 4, -3), (np.array([-0.17871288, -0.17999766]), 1, -4), (np.array([-0.31176417, -0.02954987]), -3, -4), (np.array([-0.47263947,  0.20044825]), -6, -4), (np.array([-0.59588791,  0.33722084]), -8, -4), (np.array([-0.71232924,  0.1695367 ]), -7, -7), (np.array([-0.72397451, -0.10651083]), -5, -9), (np.array([-0.63786382, -0.28111557]), -2, -9), (np.array([-0.31516298, -0.47833242]), 2, -7), (np.array([-0.06694517, -0.67444261]), 5, -6), (np.array([-0.51671547, -0.69831482]), 2, -10), (np.array([-0.06073953, -0.49829855]), 4, -5), (np.array([-0.34573393, -0.24804404]), -1, -6), (np.array([-0.35006418,  0.27358424]), -6, -2), (np.array([-0.60092873,  0.56441755]), -10, -2)]

tick_counter = 0

path = deque(maxlen=250)
force_1_queue = deque(maxlen=30)
force_2_queue = deque(maxlen=30)


def record_calibration_point():
    point = (simulation.get_input(), motor1.step, motor2.step)
    calibration_points.append(point)


def calibrate():
    points = calibration_points * 10
    np.random.shuffle(points)
    for pos, step1, step2 in points:
        e = 0.001
        gradient = calibrator._calculate_gradient(step1, step2, pos)
        print(gradient)
        break
        print(loss, initial_motor_1.pos, initial_motor_2.pos)


#  MAIN LOOP
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                motor1.left()
                simulation.move = False
            if event.key == pygame.K_RIGHT:
                motor1.right()
                simulation.move = False
            if event.key == pygame.K_UP:
                motor2.left()
                simulation.move = False
            if event.key == pygame.K_DOWN:
                motor2.right()
                simulation.move = False
            if event.key == pygame.K_p:
                record_calibration_point()
            if event.key == pygame.K_c:
                calibrate()
            if event.key == pygame.K_o:
                print(calibration_points)
            if event.key == pygame.K_ESCAPE:
                running = False
        # Move target
        if event.type == pygame.MOUSEBUTTONUP:
            target = camera.world_pos(pygame.mouse.get_pos())
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

    #calibrator.tick(tick_counter)

    calculated_pos = mech_controller.calculate_cords(motor1.step, motor2.step)

    # DRAWING
    screen.fill((255, 255, 255))

    if len(path) > 1:
        pygame.draw.lines(screen, (50, 50, 50), False, path)

    for pos, _, _ in calibration_points:
        pygame.draw.circle(screen, (0, 0, 0), camera.pixel(pos), 4, 2)

    simulation.draw(screen, camera)
    pygame.draw.circle(screen, (110, 110, 200), camera.pixel(
        target), camera.pixel_len(.1), 2)

    pygame.draw.circle(screen, (200, 110, 110), camera.pixel(
        calculated_pos), camera.pixel_len(.11), 2)

    debug.draw(screen)

    pygame.display.flip()
    clock.tick(250)

pygame.quit()
