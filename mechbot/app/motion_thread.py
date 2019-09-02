import logging
import struct
import enum
import threading
import time
import queue

import serial
import numpy as np

from mechbot.controller.calibration import CalibrationHelper
from mechbot.controller.device import VirtualDevice, StepperMotor
from mechbot.controller.interface import SerialControllerInterface
from mechbot.controller.simulator import MechanicalSimulator, SimulatorThread
from mechbot.utils import vector_utils


@enum.unique
class DeviceStatusEnum(enum.Enum):
    INITIALIZED = enum.auto()
    AWAITING_CONNECTION = enum.auto()
    CONNECTED = enum.auto()
    CALIBRATING = enum.auto()
    CALIBRATED = enum.auto()
    TARGET = enum.auto()


class MotionThread(threading.Thread):
    device: VirtualDevice

    def __init__(self, run_until, config):
        super(MotionThread, self).__init__(name="MotionThread")
        self.run_until = run_until
        self.config = config
        self.device = None
        self.status_listeners = []
        self.detections_queue = queue.Queue(maxsize=5)
        self.calibrated = False

    def run(self):
        with self.setup_interface_context() as interface:
            self.send_status(DeviceStatusEnum.AWAITING_CONNECTION)
            while self.active() and not interface.is_alive():
                self.send_status(DeviceStatusEnum.AWAITING_CONNECTION)
                time.sleep(.1)
            if not self.active():
                logging.info("exit")
                return  # exit on shutdown during initialization

            self.send_status(DeviceStatusEnum.CONNECTED)

            calibrator = self.setup_calibrator(interface)
            calibrator.start()

            # Calibrate
            while self.active() and not calibrator.is_done():
                self.send_status(DeviceStatusEnum.CALIBRATING)
                calibrator.tick()
                time.sleep(.01)
            if not self.active():
                logging.info("exit")
                return

            self.device = calibrator.get_result()
            # # Some code to set a debug device
            # steps = self.config.motor_steps
            # motor1 = StepperMotor((2., 2.), steps, 1, -2.356)
            # motor2 = StepperMotor((-2., 2.), steps, 1, -0.785)
            # self.device = VirtualDevice(motor1, motor2, .2, .1)
            self.calibrated = True
            self.send_status(DeviceStatusEnum.CALIBRATED, self.device)

            def motors_moved(step1, step2):
                self.device.motor1.step = step1
                self.device.motor2.step = step2

            interface.add_motion_listener(motors_moved)

            while self.active():
                # Wait for next detection
                try:
                    t_start, detections = self.detections_queue.get(timeout=.2)
                except queue.Empty:
                    continue

                pos, s1, s2 = self.calculate_target(t_start, detections)
                if pos is not None:
                    self.send_status(DeviceStatusEnum.TARGET, pos, s1, s2)

                    interface.cmd_goto(s1, s2)
        logging.info("exit")

    def calculate_target(self, t_start, detections):
        # Position in -0.5 to 0.5 coordinate system for every enemy
        centers = [np.array([(x_min + x_max), (y_min + y_max)]) / 2. - 0.5 for
                   (y_min, x_min, y_max, x_max), class_id in detections if
                   class_id == self.config.target_class_id]
        monitor_size = np.array([self.config.monitor_width,
                                 self.config.monitor_height])

        if not centers:
            return None, None, None

        # Offset in pixels from crosshair
        offsets = [pos * monitor_size for pos in centers]
        nearest = min(offsets, key=vector_utils.vec_len)

        # Calculate wanted joystick deflection
        deflection = (vector_utils.vec_len(nearest)
                      / self.config.full_deflection_dist)
        if deflection > self.config.full_deflection:
            deflection = self.config.full_deflection
        angle = np.arctan2(nearest[1], nearest[0])
        target = vector_utils.dir_vec(angle) * deflection

        s1, s2 = self.device.calculate_steps(target)

        return target, s1, s2

    def setup_interface_context(self):
        if self.config.use_simulator:
            steps = self.config.motor_steps
            motor1 = StepperMotor((2., 2.), steps, 1, -2.356)
            motor2 = StepperMotor((-2., 2.), steps, 1, -0.785)
            device = VirtualDevice(motor1, motor2, .2, .1)
            sim = MechanicalSimulator(device, stick_force=.099)
            return SimulatorThread(sim, self.config.simulator_dt)
        else:
            return SerialControllerInterface(self.config.serial_port,
                                             self.config.serial_baud,
                                             self.config.joystick_number,
                                             self.config.joystick_axis_x,
                                             self.config.joystick_axis_y)

    def setup_calibrator(self, interface):
        options = vars(self.config)
        # Extracts all options named "calib_" from config and uses them as key
        # word args
        kwargs = dict(
            [(k[6:], options[k]) for k in options if k.startswith("calib_")])

        return CalibrationHelper(interface, self.config.motor_steps, **kwargs)

    def active(self):
        return not self.run_until.is_set()

    def push_detections(self, frame, detections, timings):
        if self.calibrated:
            self.detections_queue.put((timings.latest_start_time(),
                                       detections))

    def send_status(self, *args):
        for listener in self.status_listeners:
            listener(*args)

    def add_status_listener(self, listener):
        self.status_listeners.append(listener)
