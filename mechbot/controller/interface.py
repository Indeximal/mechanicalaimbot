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
        self.alive_2 = False
        self.motion_listeners = []

    def connection_made(self, transport):
        super(SerialProtocol, self).connection_made(transport)
        logging.info("connection made")
        transport.write(b"\xff\x00")

    def handle_packet(self, packet):
        if not packet:
            logging.warning("received empty message!")
            return
        head = packet[0]
        if head == 0xff:
            logging.info("Controller alive")
            self.alive_2 = True
        elif head == 0x01:
            for listener in self.motion_listeners:
                listener(packet[1:])
            logging.debug("Motion:" + packet[1:])
        else:
            logging.warning("Unknown response from controller!")

    def connection_lost(self, exc):
        super(SerialProtocol, self).connection_lost(exc)
        self.alive_2 = False


class SerialControllerInterface:
    def __init__(self, serial_port, serial_baud, joystick_number,
                 joystick_axis_x, joystick_axis_y):
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(joystick_number)
        self.joystick.init()
        self.axis_x = joystick_axis_x
        self.axis_y = joystick_axis_y

        self.serial_port = serial_port
        self.serial_baud = serial_baud

    def __enter__(self):
        ser = serial.Serial(self.serial_port, self.serial_baud)
        self.serial_thread = serial.threaded.ReaderThread(ser, SerialProtocol)
        self.serial_thread.daemon = True
        self.serial_thread.name = "SerialThread"
        self.protocol = self.serial_thread.__enter__()
        return self

    def __exit__(self, *args):
        self.serial_thread.__exit__(*args)

    def is_alive(self):
        return self.protocol.alive_2

    def cmd_goto(self, s1, s2):
        self.serial_thread.write(struct.pack("BBBB", 0x01, s1, s2, 0x00))

    def get_input(self):
        axis_x = self.joystick.get_axis(self.axis_x)
        axis_y = self.joystick.get_axis(self.axis_y)
        return np.array((axis_x, axis_y))

    def add_motion_listener(self, listener):
        self.protocol.motion_listeners.append(listener)
