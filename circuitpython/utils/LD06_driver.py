'''
CircuitPython
'''

import board                    # type: ignore
from ulab import numpy as np    # type: ignore
from pwmio import PWMOut        # type: ignore
from busio import UART          # type: ignore


class LD06:
    def __init__(self, uart_pin, pwm_pin, pwm_dc=0.4):
        self.uart_pin = uart_pin
        self.pwm_pin = pwm_pin
        self.pwm_dc = pwm_dc
        self.pwm_dc_16bit = 0
        self.pwm = PWMOut(self.pwm_pin, frequency=20000)
        self.set_pwm_dc(self.pwm_dc)
        
        self.start_byte = 0x54
        self.package_len = 46
        self.output_len = 50  # x12 2D coordinates
        self.len_byte = 44  # 0x2c
        self.dtype = np.float
        
        self.angle_offset = np.pi / 2  # 90°

        # init ouput array
        self.points_2d = np.empty((0,3), dtype=self.dtype)
   

    def set_pwm_dc(self, dc):
        self.pwm_dc = dc
        self.pwm_dc_16bit = int(self.pwm_dc * 65534)
        self.pwm.duty_cycle = self.pwm_dc_16bit
    

    def LD06_serial(self, Rx_pin):
        return UART(None, Rx_pin, baudrate=230400, bits=8, parity=None, stop=1)


    def decode(self, data_package):
        speed = int.from_bytes(data_package[0:2], 'big') / 100
        FSA = int.from_bytes(data_package[2:4], 'big') / 100
        LSA = int.from_bytes(data_package[40:42], 'big') / 100
        timestamp = int.from_bytes(data_package[42:44], 'big') % 3000
        CS = int.from_bytes(data_package[44:45], 'big')
        
        angleStep = (LSA - FSA) / 11 if LSA - FSA > 0 else (LSA + 360 - FSA) / 11
        
        angle_array     = np.zeros(12)
        distance_array  = np.zeros(12)
        luminance_array = np.zeros(12)

        # data = int.from_bytes(data_package[4:40], 'big') / 100
        for i in range(12):
            angle_array[i]     = (angleStep * i + FSA) % 360
            j = i*3
            distance_array[i]  = int.from_bytes(data_package[4+j : 6+j], 'big') / 100
            luminance_array[i] = int.from_bytes(data_package[6+j : 7+j], 'big')
        
        return speed, timestamp, angle_array, distance_array, luminance_array


    def polar2cartesian(self, angles, distances, offset):
        angles = angles + offset
        x_array = distances * -np.cos(angles)
        y_array = distances * np.sin(angles)
        return x_array, y_array


    def read(self):   
        with self.LD06_serial(self.uart_pin) as serial_connection:
            i = 0
            self.points_2d = np.empty((0,3), dtype=self.dtype)

            while True:
                # Detect the start of the package
                while True:
                    start_byte = serial_connection.read(1)
                    if start_byte is not None and int.from_bytes(start_byte, 'big') == self.start_byte:
                        break

                data_package = serial_connection.read(self.package_len)

                
                # Error handling
                if data_package[0] != self.len_byte:  # 0x2c
                    # print("[WARNING] no Data Length Byte (0x2c) found")
                    continue
                # Check the length of data_package
                elif len(data_package) < self.package_len:
                    # print("[WARNING] Incomplete data package")
                    continue
                
                # drop first byte (0x2c -> dlength) and decode remaining data
                speed, timestamp, angle_batch, distance_batch, luminance_batch = self.decode(data_package[1:])

                # cartesian coordinates
                x_batch, y_batch = self.polar2cartesian(angle_batch, distance_batch, self.angle_offset)

                # stack table and append to points_2d
                points_batch = self.hstack(x_batch, y_batch, luminance_batch)
                self.points_2d = self.vstack(self.points_2d, points_batch)
                
                # return points_2d
                if i % self.output_len == self.output_len - 1:
                    return speed, timestamp, self.points_2d

                i += 1


    @staticmethod
    def hstack(a, b, c):
        return np.concatenate((a.reshape((-1,1)), b.reshape((-1,1)), c.reshape((-1,1))), axis=-1)
    
    @staticmethod
    def vstack(a, b):
        return np.concatenate((a, b), axis=0)



if __name__ == "__main__":
    
    lidar = LD06(board.GP1, board.GP2, pwm_dc=0.3)

    while True:
        speed, timestamp, points_2d = lidar.read()
        print(len(points_2d), points_2d[0:5])
