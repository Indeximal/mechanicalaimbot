from pathlib import Path

import configargparse
import numpy as np

from mechbot import resources
from mechbot.controller.calibration import AlignmentHelper
from mechbot.controller.device import StepperMotor, VirtualDevice
from mechbot.controller.interface import SerialControllerInterface
from mechbot.controller.simulator import MechanicalSimulator, SimulatorThread
from mechbot.utils import vector_utils
from mechbot.utils.fields import CsgoTeamEnum


def parse_config():
    parser = configargparse.ArgParser(
        default_config_files=[resources.DEFAULT_CONFIG])

    parser.add("-c", "--config", is_config_file=True, help="config file path")
    parser.add("--serial_port", type=str, required=True)
    parser.add("--serial_baud", type=int, required=True)
    parser.add("--display_width", type=int, required=True)
    parser.add("--display_height", type=int, required=True)
    parser.add("--inference_graph", type=str, required=True)
    parser.add("--inference_input_format", type=str, required=True,
               choices=["RGB", "BGRA"])
    parser.add("--display_name", type=str, required=True)
    parser.add("--display_fps", type=int, required=True)
    parser.add("--score_thresh", type=float, required=True)
    parser.add("--monitor_number", type=int, required=True)
    parser.add("--monitor_width", type=int, required=True)
    parser.add("--monitor_height", type=int, required=True)
    parser.add("--rect_width", type=int, required=True)
    parser.add("--joystick_number", type=int, required=True)
    parser.add("--joystick_axis_x", type=int, required=True)
    parser.add("--joystick_axis_y", type=int, required=True)
    parser.add("--t_head_id", type=int, required=True)
    parser.add("--t_body_id", type=int, required=True)
    parser.add("--ct_head_id", type=int, required=True)
    parser.add("--ct_body_id", type=int, required=True)
    parser.add("--step_shift", type=int, required=True)
    parser.add("--motor_steps", type=int, required=True)
    parser.add("--device_gap", type=float, required=True)
    parser.add("--controller_deadzone", type=float, required=True)
    parser.add("--movement_threshold", type=float, required=True)
    parser.add("--joystick_input_dt", type=float, required=True)
    parser.add("--initial_camera_constant", type=float, required=True)
    parser.add("--new_camera_constant_weight", type=float, required=True)
    parser.add("--motion_deadzone", type=float, required=True)
    parser.add("--deadzone_avoidance_radius", type=float, required=True)
    parser.add("--use_velocity_algorithm", action="store_true")
    parser.add("--motor_radius", type=float, required=True)
    parser.add("--device_size", type=float, required=True)
    parser.add("--motor1_angle", type=float, required=True)
    parser.add("--motor2_angle", type=float, required=True)
    parser.add("--body_target_height", type=float, required=True)
    parser.add("--use_simulator", action="store_true")
    parser.add("--use_debug_detection", action="store_true")
    parser.add("--invert_y", action="store_true")
    parser.add("--debug_output", action="store_true")
    parser.add("--simulator_dt", type=float, required=True)
    parser.add("--calib_max_deflection", type=float, required=True)
    parser.add("--calib_center_threshold", type=float, required=True)
    parser.add("--joystick_radius", type=float, required=True)
    parser.add("--calib_motion_threshold", type=float, required=True)
    parser.add("--calib_wait_ticks", type=int, required=True)
    parser.add("--calib_wait_duration", type=float, required=True)
    parser.add("--full_deflection", type=float, required=True)
    parser.add("--camera_shift_influence", type=float, required=True)
    parser.add("--controller_speed", type=float, required=True)
    parser.add("--pink_class_id", type=int, required=True)
    parser.add("--pink_slow_down", type=float, required=True)
    parser.add("--aim_dampening", type=float, required=True)
    parser.add("--pink_min_area", type=int, required=True)
    parser.add("--text_size", type=int, required=True)
    parser.add("--text_color", type=int, action="append",
               required=True)
    parser.add("--text_bg", type=int, action="append",
               required=True)
    parser.add("--rect_color_per_class", type=int, action="append",
               required=True)
    parser.add("--default_team", required=True,
               choices=[team.value for team in CsgoTeamEnum])

    # Joystick Sensitivity curve approximation
    parser.add("--sigmoid_max", type=float, required=True)
    parser.add("--sigmoid_steepness", type=float, required=True)
    parser.add("--sigmoid_offset", type=float, required=True)

    config, _ = parser.parse_known_args()

    # Handle special cases for paths and 2d arrays in options
    config.rect_color_per_class = np.array(
        config.rect_color_per_class).reshape(-1, 3)

    if config.inference_graph.startswith("resources."):
        inference_path = vars(resources)[config.inference_graph[10:]]
    else:
        inference_path = str(Path(config.inference_graph))
    config.inference_graph = inference_path

    return config


def setup_interface_context(config):
    """prepares the interface context manager to be used based on config"""
    if config.use_simulator:
        device = setup_device(config)
        sim = MechanicalSimulator(device, stick_force=.099)
        return SimulatorThread(sim, config.simulator_dt)
    else:
        return SerialControllerInterface(config.serial_port,
                                         config.serial_baud,
                                         config.joystick_number,
                                         config.joystick_axis_x,
                                         config.joystick_axis_y,
                                         config.step_shift)


def setup_calibrator(config, interface):
    options = vars(config)
    # Extracts all options named "calib_" from config and uses them as key
    # word args
    kwargs = dict(
        [(k[6:], options[k]) for k in options if k.startswith("calib_")])

    return AlignmentHelper(interface, **kwargs)


def setup_device(config):
    motors = []
    for angle in [config.motor1_angle, config.motor2_angle]:
        pos = vector_utils.dir_vec(angle) * config.motor_radius
        motor = StepperMotor(pos, config.motor_steps, 1, 0.0)
        motor.align = np.arctan2(-motor.pos[1], -motor.pos[0])
        motors.append(motor)

    gap = config.device_gap + config.joystick_radius
    device = VirtualDevice(motors[0], motors[1], gap,
                           config.joystick_radius)
    return device


def setup_sensitivity_curve(config):
    """returns a function with signature (deflection) -> (angular speed)"""
    a = config.sigmoid_steepness
    b = config.sigmoid_max
    c = config.sigmoid_offset

    def sigmoid(x):
        return b / (1 + np.exp(-a * x + c))

    return lambda x: np.sign(x) * sigmoid(np.abs(x))


def setup_inverse_sensitivity_curve(config):
    """returns a function with signature (angular speed) -> (deflection)"""
    a = config.sigmoid_steepness
    b = config.sigmoid_max
    c = config.sigmoid_offset

    def inv_sigmoid(y):
        return (-np.log(b / y - 1) + c) / a

    def curve(y):
        if y > 0:
            x = inv_sigmoid(y)
            return np.maximum(x, 0)
        elif y == 0:
            return 0
        else:
            x = -inv_sigmoid(-y)
            return np.minimum(x, 0)

    return curve
