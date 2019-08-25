import numpy as np
import pygame
import serial
import serial.threaded


# Runs in "Serial" Thread
class SerialProtocol(serial.threaded.Packetizer):
    def __init__(self):
        super(SerialProtocol, self).connection_made(transport)
        self.alive = False

    def connection_made(self, transport):
        super(SerialProtocol, self).connection_made(transport)
        transport.write(b"\xff\x00")

    def handle_packet(self, packet):
        head = packet[0]
        if head == 0xff:
            print("Controller alive")
            self.alive = True
        elif head == 0x01:
            print("Motion:" + packet[1:])
        else:
            print("Unknown response from controller!")

    def connection_lost(self, exc):
        super(SerialProtocol, self).connection_lost(exc)
        self.alive = False 


class SerialControllerInterface:
    def __init__(self, joystick_number, serial_port, serial_baud):
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(joystick_number)
        self.joystick.init()

        self.serial_port = serial_port
        self.serial_baud = serial_baud

    def __enter__(self):
        ser = serial.Serial(self.serial_port, self.serial_baud)
        self.serial_thread = serial.threaded.ReaderThread(ser, SerialProtocol)
        self.serial_thread.daemon = True
        transport, self.protocol = self.serial_thread.connect()
        print("transport == ser:", transport == ser)

    def __exit__(self, *args):
        self.serial_thread.close()

    def is_alive():
        return self.protocol.alive

    def cmd_goto(self, s1, s2):
        self.serial_thread.write(struct.pack("BB", s1, s2))

    def get_input(self):
        axis_x = self.joystick.get_axis(3)
        axis_y = self.joystick.get_axis(4)
        return np.array((axis_x, axis_y))
