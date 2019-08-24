import struct
import threading
import time
import queue

import serial

from mechbot.controller.interface import ControllerInterface
 

class MotionThread(threading.Thread):
    def __init__(self, run_until, serial_port, serial_baud):
        super(MotionThread, self).__init__(name="Motion-Thread")
        self.run_until = run_until
        self.serial_port = serial_port
        self.serial_baud = serial_baud
        self.calibrated_listeners = []
        self.target_listeners = []
        self.detections_queue = queue.Queue(maxsize=5)
        self.calibrated = False

    def active(self):
        return not self.run_until.is_set()

    def run(self):
        with SerialConnectionThread(serial_port, serial_baud) as connection:
            # todo timeout
            while self.active() and not connection.is_alive():
                time.sleep(.01)

            interface = connection.get_interface()

            # Calibrate
            calibrator = CalibrationHelper(interface, motor_steps, max_deflection=.3,
                 center_threshold=.05, circle_1_radius=.8, circle_2_radius=.5,
                 angular_step_1=.4, angular_step_2=.8, do_optimization=True,
                 motion_threshold=.03, wait_ticks=20, wait_duration=.4)

            while self.active() and not calibrator.is_done():
                calibrator.tick()
                time.sleep(.01)

            device = calibrator.get_result()

            def motors_moved(step1, step2):
                device.motor1.step = step1
                device.motor2.step = step2

            connection.add_motion_listener(motors_moved)

            self.calibrated = True

            while self.active():
                time.sleep(1 / 60)  # Limit thread while testing


    def push_detections(self, frame, detections, timings):
        if self.calibrated:
            self.detections_queue.put((timings.start_time, detections))

    def add_calibrated_listener(self, listener):
        self.calibrated_listeners.append(listener)

    def add_target_listener(self, listener):
        self.target_listeners.append(listener)



