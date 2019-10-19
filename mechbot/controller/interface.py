import logging
import struct

import numpy as np
import pygame
import serial
import serial.threaded


# Runs in "Serial" Thread
class SerialProtocol(serial.threaded.Packetizer):
    def __init__(self):
        super().__init__()
        self.alive = False
        self.shift = 0
        self.motion_listeners = []

    def connection_made(self, transport):
        super(SerialProtocol, self).connection_made(transport)
        logging.info("connection made")
        self.alive = True
        transport.write(struct.pack("BBBB", 0xff, 0xff, 0xff, 0x00))

    def handle_packet(self, packet):
        # logging.debug(packet)
        if not packet:
            logging.warning("received empty message!")
            return
        head = packet[0]
        if head == 0xff:
            logging.info("Controller alive")
            self.alive = True
        elif head == 0xfe:
            pass  # successfully received message
        elif head == 0x01:
            if len(packet) != 3:
                logging.warning("Motion package has invalid length")
                return
            for listener in self.motion_listeners:
                step1 = packet[1] - self.shift
                step2 = packet[2] - self.shift
                listener(step1, step2)
            # logging.debug("Motion:" + packet[1:])
        else:
            logging.warning("Unknown response from controller!")

    def connection_lost(self, exc):
        super(SerialProtocol, self).connection_lost(exc)
        self.alive = False


class SerialControllerInterface:
    def __init__(self, serial_port, serial_baud, joystick_number,
                 joystick_axis_x, joystick_axis_y, shift):
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(joystick_number)
        self.joystick.init()
        self.axis_x = joystick_axis_x
        self.axis_y = joystick_axis_y
        self.shift = shift

        self.serial_port = serial_port
        self.serial_baud = serial_baud

        self.last_1 = 0
        self.last_2 = 0

    def __enter__(self):
        ser = serial.Serial(self.serial_port, self.serial_baud)
        self.serial_thread = serial.threaded.ReaderThread(ser, SerialProtocol)
        self.serial_thread.daemon = True
        self.serial_thread.name = "SerialThread"
        self.protocol = self.serial_thread.__enter__()
        self.protocol.shift = self.shift
        return self

    def __exit__(self, *args):
        self.serial_thread.__exit__(*args)

    def is_alive(self):
        return self.protocol.alive

    def cmd_goto(self, step_1, step_2):
        if step_1 == self.last_1 and step_2 == self.last_2:
            return
        self.last_1 = step_1
        self.last_2 = step_2
        s1 = step_1 + self.shift
        s2 = step_2 + self.shift
        self.serial_thread.write(struct.pack("BBBB", 0x01, s1, s2, 0x00))
        logging.debug((step_1, step_2))

    def get_input(self):
        axis_x = self.joystick.get_axis(self.axis_x)
        axis_y = self.joystick.get_axis(self.axis_y)
        return np.array((axis_x, axis_y))

    def add_motion_listener(self, listener):
        self.protocol.motion_listeners.append(listener)
