import logging
import threading
import time
import queue

import numpy as np

from mechbot.controller.calibration import CalibrationHelper, AlignmentHelper
from mechbot.controller.device import VirtualDevice, StepperMotor
from mechbot.controller.interface import SerialControllerInterface
from mechbot.controller.joystick_input import JoystickInputThread
from mechbot.controller.simulator import MechanicalSimulator, SimulatorThread
from mechbot.utils import vector_utils
from mechbot.utils.fields import CsgoTeamEnum
from mechbot.utils.fields import DeviceStatusEnum


class MotionThread(threading.Thread):
    def __init__(self, run_until, config):
        super(MotionThread, self).__init__(name="MotionThread")
        self.run_until = run_until
        self.config = config
        self.device = None
        self.status_listeners = []
        self.detections_queue = queue.Queue(maxsize=5)
        self.calibrated = False
        self.team = CsgoTeamEnum(config.default_team)

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

            time.sleep(.5)  # Wait a little

            # TODO: Figure out which motor is which
            # calibrator = self.setup_calibrator(interface)
            # calibrator.start()
            #
            # # Calibrate
            # while self.active() and not calibrator.is_done():
            #     self.send_status(DeviceStatusEnum.CALIBRATING)
            #     calibrator.tick()
            #     time.sleep(.01)
            # if not self.active():
            #     logging.info("exit")
            #     return

            self.device = self.device_from_config()
            # calibrator.apply_to_device(self.device)

            self.calibrated = True
            self.send_status(DeviceStatusEnum.CALIBRATED, self.device)

            def motors_moved(step1, step2):
                self.device.motor1.step = step1
                self.device.motor2.step = step2

            interface.add_motion_listener(motors_moved)

            joystick_thread = JoystickInputThread(self.run_until, self.config,
                                                  interface)
            joystick_thread.start()

            last_target = None
            last_motion = None
            last_joystick = np.zeros(2)
            camera_constant = self.config.initial_camera_constant

            # Loop: Wait for next detection, process, repeat
            while self.active():
                try:
                    # Timeout and try needed in the case of a shutdown
                    avg_t, detections = self.detections_queue.get(timeout=.2)
                except queue.Empty:
                    continue

                targets = self.process_detections(detections)
                joystick_motion = joystick_thread.get_and_reset_total()
                camera_shift = joystick_motion * camera_constant

                # No target to aim at
                if not targets:
                    last_target = None
                    last_motion = None
                    last_joystick = joystick_motion
                    pos = np.zeros(2)
                    self.send_status(DeviceStatusEnum.TARGET, pos, 0, 0)
                    interface.cmd_goto(0, 0)
                    continue

                # TODO: multiple targets
                target = min(targets, key=vector_utils.vec_len)

                # New target: velocity assumed to be 0
                if last_target is None:
                    last_motion = None
                    needed_input = (-1 / camera_constant
                                    * (target + camera_shift))
                # Existing target: assume velocity is constant
                else:
                    target_motion = last_target - target
                    velocity = target_motion - last_joystick * camera_constant
                    needed_input = (- 1 / camera_constant
                                    * (target + camera_shift + velocity))

                    if last_motion is not None:
                        # new_cam_const = ((target_motion - last_motion)
                        #                  / (joystick_motion - last_joystick))
                        delta_m = target_motion - last_motion
                        if vector_utils.vec_len(delta_m) != 0.0:
                            delta_i = joystick_motion - last_joystick
                            new_cam_const = (np.dot(delta_m, delta_i)
                                             / vector_utils.vec_len(delta_m))
                            logging.debug("k=%s", new_cam_const)
                            # Update cam constant based on a weighted average
                            alpha = self.config.new_camera_constant_weight
                            camera_constant = ((1 - alpha) * camera_constant
                                               + alpha * new_cam_const)

                    last_motion = target_motion

                last_target = target

                # Use the needed joystick input to calculate where to move to
                # to get the desired motion
                pos, s1, s2 = self.calculate_target_step(needed_input, avg_t)

                self.send_status(DeviceStatusEnum.TARGET, pos, s1, s2)
                interface.cmd_goto(s1, s2)

                last_target = target
                last_joystick = joystick_motion

        logging.info("exit")

    def process_detections(self, detections):
        """detections has form [(ymin, xmin, ymax, xmax), classID]
        returns a list of pixel offsets from the center
        """

        # Get IDs for the enemy team
        if self.team == CsgoTeamEnum.TERRORISTS:
            head_id = self.config.ct_head_id
            body_id = self.config.ct_body_id
        else:  # self.team == CsgoTeamEnum.COUNTER_TERRORISTS:
            head_id = self.config.t_head_id
            body_id = self.config.t_body_id

        heads = [box for box, classID in detections if classID == head_id]
        # TODO: use body information
        bodies = [box for box, classID in detections if classID == body_id]

        # range [-0.5, 0.5]
        centers = [np.array([(x_min + x_max), (y_min + y_max)]) / 2. - 0.5 for
                   (y_min, x_min, y_max, x_max) in heads]

        monitor_size = np.array([self.config.monitor_width,
                                 self.config.monitor_height])

        # range [-width/2, width/2] and [-height/2, height/2]
        offsets = [pos * monitor_size for pos in centers]

        return offsets

    def calculate_target_step(self, needed_motion, duration):
        pos = needed_motion / duration
        # TODO: Compensate for suboptimal starting point

        # Stay out of the deadzone
        deadzone = self.config.controller_deadzone
        x, y = pos
        if self.config.motion_deadzone < abs(x) < deadzone:
            x = deadzone if x > 0 else -deadzone
        if self.config.motion_deadzone < abs(y) < deadzone:
            y = deadzone if y > 0 else -deadzone
        pos = np.array([x, y])

        if vector_utils.vec_len(pos) > self.config.full_deflection:
            # logging.debug("deflection clamped: not fast enough")
            pos = vector_utils.unit_vec(pos) * self.config.full_deflection

        s1, s2 = self.device.calculate_steps(pos)
        return pos, s1, s2

    @DeprecationWarning
    def calculate_target(self, t_start, detections):
        # Position in -0.5 to 0.5 coordinate system for every enemy
        centers = [np.array([(x_min + x_max), (y_min + y_max)]) / 2. - 0.5 for
                   (y_min, x_min, y_max, x_max), class_id in detections if
                   class_id == self.config.target_class_id]
        monitor_size = np.array([self.config.monitor_width,
                                 self.config.monitor_height])

        if not centers:
            return None

        # Offset in pixels from crosshair
        offsets = [pos * monitor_size for pos in centers]
        nearest = min(offsets, key=vector_utils.vec_len)

        # Calculate wanted joystick deflection
        deflection = vector_utils.vec_len(nearest) / (
            (self.config.full_deflection_dist - self.config.min_deflection)
            + self.config.min_deflection)

        if deflection > self.config.full_deflection:
            deflection = self.config.full_deflection
        # invert y if necessary
        angle = np.arctan2((-1 if self.config.invert_y else 1) * nearest[1],
                           nearest[0])
        target = vector_utils.dir_vec(angle) * deflection

        s1, s2 = self.device.calculate_steps(target)

        return target, s1, s2

    def setup_interface_context(self):
        """prepares the interface context manger to be used based on config"""
        if self.config.use_simulator:
            device = self.device_from_config()
            sim = MechanicalSimulator(device, stick_force=.099)
            return SimulatorThread(sim, self.config.simulator_dt)
        else:
            return SerialControllerInterface(self.config.serial_port,
                                             self.config.serial_baud,
                                             self.config.joystick_number,
                                             self.config.joystick_axis_x,
                                             self.config.joystick_axis_y,
                                             self.config.step_shift)

    def setup_calibrator(self, interface):
        options = vars(self.config)
        # Extracts all options named "calib_" from config and uses them as key
        # word args
        kwargs = dict(
            [(k[6:], options[k]) for k in options if k.startswith("calib_")])

        return AlignmentHelper(interface, **kwargs)

    def device_from_config(self):
        motors = []
        for angle in [self.config.motor1_angle, self.config.motor2_angle]:
            pos = vector_utils.dir_vec(angle) * self.config.motor_radius
            motor = StepperMotor(pos, self.config.motor_steps, 1, 0.0)
            motor.align = np.arctan2(-motor.pos[1], -motor.pos[0])
            motors.append(motor)

        gap = self.config.device_gap + self.config.joystick_radius
        device = VirtualDevice(motors[0], motors[1], gap,
                               self.config.joystick_radius)
        return device

    def active(self):
        return not self.run_until.is_set()

    def push_detections(self, frame, detections, timings):
        if self.calibrated:
            self.detections_queue.put((timings.avg_lap_time(),
                                       detections))

    def send_status(self, *args):
        for listener in self.status_listeners:
            listener(*args)

    def add_status_listener(self, listener):
        self.status_listeners.append(listener)

    def team_selection_listener(self, team):
        self.team = team
