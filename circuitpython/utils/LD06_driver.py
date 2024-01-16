'''
CircuitPython
'''

import board                    # type: ignore
from ulab import numpy as np    # type: ignore
from pwmio import PWMOut        # type: ignore
from busio import UART          # type: ignore

from ulab_utils import hstack, vstack


class LD06:
    def __init__(self, uart_pin, pwm_pin, pwm_dc=0.4):
        # pwm
        self.pwm_pin = pwm_pin
        self.pwm_dc = pwm_dc
        self.pwm_dc_16bit = 0
        self.pwm = PWMOut(self.pwm_pin, frequency=20000)
        self.set_pwm_dc(self.pwm_dc)

        # angle offset
        self.offset = np.pi / 2  # 90°

        # serial
        self.uart_pin = uart_pin
        self.start_byte = 0x54
        self.package_len = 46
        self.output_len = 50  # x12 2D coordinates
        self.len_byte = 44  # 0x2c
        self.dtype = np.float
        
        # init ouput array
        self.points_2d = np.empty((0,3), dtype=self.dtype)
   

    def set_pwm_dc(self, dc):
        self.pwm_dc = dc
        self.pwm_dc_16bit = int(self.pwm_dc * 65534)
        self.pwm.duty_cycle = self.pwm_dc_16bit
    

    @staticmethod
    def init_serial(Rx_pin):
        return UART(None, Rx_pin, baudrate=230400, bits=8, parity=None, stop=1)


    def read(self):   
        with self.init_serial(self.uart_pin) as serial_connection:
            i = 0
            self.points_2d = np.empty((0,3), dtype=self.dtype)

            while True:
                # Detect the start of the package
                while True:
                    start_byte = serial_connection.read(1)
                    if start_byte is not None and int.from_bytes(start_byte, 'big') == self.start_byte:
                        break

                byte_array = serial_connection.read(self.package_len)

                
                # Error handling
                if byte_array[0] != self.len_byte:  # 0x2c
                    print("[WARNING] no Data Length Byte (0x2c) found")
                    continue
                # Check the length of byte_array
                elif len(byte_array) < self.package_len:
                    print("[WARNING] Incomplete data package")
                    continue
                
                # drop first byte (0x2c -> dlength) and decode remaining data
                speed, timestamp, angle_batch, distance_batch, luminance_batch = self.decode(byte_array[1:])

                # cartesian coordinates
                x_batch, y_batch = self.polar2cartesian(angle_batch, distance_batch, self.offset)

                # stack table and append to points_2d
                points_batch = hstack(x_batch, y_batch, luminance_batch)
                self.points_2d = vstack(self.points_2d, points_batch)
                
                # return points_2d
                if i % self.output_len == self.output_len - 1:
                    return speed, timestamp, self.points_2d

                i += 1


    @staticmethod
    def decode(byte_array):
        dlength = 12  # byte_array[46] & 0x1F
        speed = int.from_bytes(byte_array[0:2], 'big') / 100
        FSA = int.from_bytes(byte_array[2:4], 'big') / 100
        LSA = int.from_bytes(byte_array[40:42], 'big') / 100
        timestamp = int.from_bytes(byte_array[42:44], 'big') % 3000
        CS = int.from_bytes(byte_array[44:45], 'big')
        
        angleStep = ((LSA - FSA) if LSA - FSA > 0 else (LSA + 360 - FSA)) / (dlength-1)
        
        angle_array     = np.zeros(dlength)
        distance_array  = np.zeros(dlength)
        luminance_array = np.zeros(dlength)

        # data = int.from_bytes(byte_array[4:40], 'big') / 100
        for i in range(dlength):
            angle_array[i]     = (angleStep * i + FSA) % 360
            j = i*3
            distance_array[i]  = int.from_bytes(byte_array[4+j : 6+j], 'big') / 100
            luminance_array[i] = int.from_bytes(byte_array[6+j : 7+j], 'big')
        
        return speed, timestamp, angle_array, distance_array, luminance_array


    @staticmethod
    def polar2cartesian(angles, distances, offset):
        angles = angles + offset
        x_array = distances * -np.cos(angles)
        y_array = distances * np.sin(angles)
        return x_array, y_array



if __name__ == "__main__":
    from time import time

    lidar = LD06(board.GP1, board.GP2, pwm_dc=0.3)

    while True:
        speed, timestamp, points_2d = lidar.read()
        print("[Received]", time(), timestamp, len(points_2d))
