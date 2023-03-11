import typing
import time
import serial

class IMUException(Exception):
    pass

class Rotor:

    initialized = False

    def __init__(self, serial_port: str):
        self.serial_port = serial_port
        self.serial = serial.Serial(serial_port, baudrate=9600, timeout=3)

        self.off()

    '''
    duty: 0 - 180
    '''
    def set(self, dutyLeft: int, dutyRight: int):
        a = bytearray(b'')
        a.append(dutyLeft)
        a.append(b' ')
        a.append(dutyRight)

        self.serial.write(a)

    def off(self):
        self.serial.write(b'0 0')
    
    def __del__(self):
        self.off()
