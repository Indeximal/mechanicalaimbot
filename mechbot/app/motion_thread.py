import logging
import threading
import time
import queue

import numpy as np

from mechbot.app import configuration
from mechbot.controller.joystick_input import JoystickInputThread
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
        with configuration.setup_interface_context(self.config) as interface:
            self.send_status(DeviceStatusEnum.AWAITING_CONNECTION)
            while self.active() and not interface.is_alive():
                self.send_status(DeviceStatusEnum.AWAITING_CONNECTION)
                time.sleep(.02)
            if not self.active():
                logging.info("exit")
                return  # exit on shutdown during initialization

            self.send_status(DeviceStatusEnum.CONNECTED)

            self.device = configuration.setup_device(self.config)

            self.calibrated = True
            self.send_status(DeviceStatusEnum.CALIBRATED, self.device)

            def motors_moved(step1, step2):
                self.device.motor1.step = step1
                self.device.motor2.step = step2

            interface.add_motion_listener(motors_moved)

            # for x in np.arange(0.1, 1, 0.1):
            #     self.move(interface, np.array([x, 0.0]))
            #     pass  # Code to test the joystick curve with debugging

            joystick_thread = JoystickInputThread(self.run_until, self.config,
                                                  interface)
            joystick_thread.start()

            last_target = None
            last_motion = None
            last_joystick = np.zeros(2)
            pre_last_joystick = np.zeros(2)
            camera_constant = self.config.initial_camera_constant

            # Loop: Wait for next detection, process, repeat
            while self.active():
                try:
                    # Timeout and try needed in the case of a shutdown
                    avg_t, detections = self.detections_queue.get(timeout=.2)
                except queue.Empty:
                    continue

                # Grab targets as points and get the joystick motion
                targets = self.process_detections(detections)
                joystick_input = joystick_thread.get_and_reset_total()
                camera_motion = - joystick_input * camera_constant  # [pxl]
                camera_motion *= self.config.camera_shift_influence

                # No target to aim at
                if not targets:
                    last_target = None
                    last_motion = None
                    pre_last_joystick = last_joystick
                    last_joystick = joystick_input
                    self.move(interface, np.zeros(2))
                    continue

                # Predict where the targets are now, because the information
                # we have is one inference frame old (~80ms). This can be done
                # by using the joystick and approximation how much the camera
                # has moved.
                targets_now = [target + camera_motion for target in targets]

                # TODO: multiple targets
                target_now = min(targets_now, key=vector_utils.vec_len)
                target = target_now - camera_motion

                # deemed to be close enough not to move
                if vector_utils.vec_len(target) < self.config.target_size:
                    last_target = None
                    last_motion = None
                    pre_last_joystick = last_joystick
                    last_joystick = joystick_input
                    self.move(interface, np.zeros(2))
                    continue

                # New target: velocity assumed to be 0
                if (not self.config.use_velocity_algorithm
                        or last_target is None):
                    needed_input = 1 / camera_constant * target_now
                    last_motion = None

                # Existing target: assume velocity is constant
                else:
                    # TODO Fix or test
                    target_motion = last_target - target
                    velocity = target_motion - last_joystick * camera_constant
                    needed_input = (1 / camera_constant
                                    * (target_now + velocity))

                    if last_motion is not None:
                        # weird calculation to get the camera constant
                        # probably not good
                        delta_i = last_joystick - pre_last_joystick
                        delta_m = target_motion - last_motion
                        if vector_utils.vec_len(delta_i) != 0.0:
                            new_cam_const = (np.dot(delta_m, delta_i)
                                             / (vector_utils.vec_len(delta_i)
                                             ** 2))
                            logging.debug("k=%s", new_cam_const)
                            # Update cam constant based on a weighted average
                            alpha = self.config.new_camera_constant_weight
                            camera_constant = ((1 - alpha) * camera_constant
                                               + alpha * new_cam_const)

                    last_motion = target_motion

                # Overshooting is a bigger problem than overshooting, so we
                # prevent it by multiplying with a constant smaller than 1.
                needed_input *= self.config.aim_dampening

                # Use the needed joystick input to calculate where to move to
                # to get the desired motion.
                pos = self.calculate_target(needed_input, avg_t,
                                            interface.get_input())
                self.move(interface, pos)

                pre_last_joystick = last_joystick
                last_joystick = joystick_input
                last_target = target

            interface.cmd_goto(0, 0)

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
        bodies = [box for box, classID in detections if classID == body_id]

        a0 = self.config.body_target_height
        a1 = 1 - a0
        body_centers = [np.array([(x_min + x_max) / 2,
                                  y_min * a0 + y_max * a1]) for
                        (y_min, x_min, y_max, x_max) in bodies]

        # range [-0.5, 0.5]
        centers = [np.array([(x_min + x_max), (y_min + y_max)]) / 2. - 0.5 for
                   (y_min, x_min, y_max, x_max) in heads]

        monitor_size = np.array([self.config.monitor_width,
                                 self.config.monitor_height])

        # range [-width/2, width/2] and [-height/2, height/2]
        offsets = [pos * monitor_size for pos in centers + body_centers]

        return offsets

    def calculate_target(self, needed_motion, timespan, current_pos):
        sensitivity_func = configuration.setup_sensitivity_curve(self.config)
        inv_func = configuration.setup_inverse_sensitivity_curve(self.config)
        max_speed = sensitivity_func(self.config.full_deflection)

        pos = []  # will be filled with x and y coords

        # for both components x and y
        for needed_mov, current_defl in zip(needed_motion, current_pos):
            current_speed = sensitivity_func(current_defl)
            needed_speed = needed_mov / timespan

            if abs(needed_speed) < self.config.movement_threshold:
                pos.append(0.0)
                continue

            # # TODO: not based on speed but deflection
            # movement_distance = abs(current_speed - needed_speed)
            # movement_time = movement_distance / self.config.controller_speed
            #
            # duration = timespan - movement_time
            # if duration < 0:
            #     duration = timespan
            #     # TODO figure out what to do
            #     logging.debug("not able to reach")
            #
            # # improve the accuracy by doing another iteration
            # # TODO only accurate when switching sides
            # needed_speed = needed_mov / duration

            needed_speed = np.clip(needed_speed, -max_speed, max_speed)
            needed_defl = inv_func(needed_speed)

            pos.append(needed_defl)

        if vector_utils.vec_len(pos) > self.config.full_deflection:
            # logging.debug("deflection clamped: not fast enough")
            pos = vector_utils.unit_vec(pos) * self.config.full_deflection

        return pos

    def move(self, interface, pos):
        if self.config.invert_y:
            pos[1] *= -1

        if pos[0] == pos[1] == 0:
            s1 = 0
            s2 = 0
        else:
            s1, s2 = self.device.calculate_steps(pos)
        current_pos = interface.get_input()
        self.send_status(DeviceStatusEnum.TARGET, pos, s1, s2, current_pos)

        interface.cmd_goto(s1, s2)

    # TODO Remove
    def calculate_target_old(self, needed_motion, duration):
        pos = needed_motion / duration

        # Stay out of the deadzone
        deadzone = self.config.controller_deadzone
        min_xy = self.config.deadzone_avoidance_radius
        x, y = pos
        if self.config.motion_deadzone < abs(x) < deadzone:
            x = min_xy if x > 0 else -min_xy
        if self.config.motion_deadzone < abs(y) < deadzone:
            y = min_xy if y > 0 else -min_xy
        pos = np.array([x, y])

        if vector_utils.vec_len(pos) > self.config.full_deflection:
            # logging.debug("deflection clamped: not fast enough")
            pos = vector_utils.unit_vec(pos) * self.config.full_deflection

        pos *= -1  # Somehow everything is inverted, so I fixed it.

        return pos

    def send_status(self, *args):
        for listener in self.status_listeners:
            listener(*args)

    def active(self):
        return not self.run_until.is_set()

    def push_detections(self, frame, detections, timings):
        if self.calibrated:
            self.detections_queue.put((timings.avg_lap_time(), detections))

    def team_selection_listener(self, team):
        self.team = team

    def add_status_listener(self, listener):
        self.status_listeners.append(listener)
