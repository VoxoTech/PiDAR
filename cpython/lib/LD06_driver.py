'''
LD06 datasheet:
https://storage.googleapis.com/mauser-public-images/prod_description_document/2021/315/8fcea7f5d479f4f4b71316d80b77ff45_096-6212_a.pdf

375 batches/s x 12 samples/batch = 4500 samples/s
'''

import numpy as np
import serial
import keyboard

try:
    from lib.platform_dependent import init_serial, get_platform, init_pwm
    from lib.file_utils import save_data
except:
    from platform_dependent import init_serial, get_platform, init_pwm
    from file_utils import save_data
    

class LD06:
    def __init__(self, port, offset=0, data_dir="data", out_len=40, format=None, visualization=None, dtype=np.float32):
        self.platform           = get_platform()

        # constants
        self.start_byte         = bytes([0x54])
        self.dlength_byte       = bytes([0x2c])
        self.dlength            = 12  # 12 samples per batch
        self.package_len        = 47  # 45 byte + 0x54 + 0x2c
        self.deg2rad            = np.pi / 180
        self.offset             = offset

        # serial
        self.port               = port
        self.serial_connection  = init_serial(port=self.port, platform=self.platform)
        self.byte_array         = bytearray()
        self.flag_2c            = False
        self.dtype              = dtype

        self.out_len           = out_len
        # preallocate batch:
        self.timestamp          = 0
        self.speed              = 0
        self.angle_batch        = np.zeros(self.dlength)
        self.distance_batch     = np.zeros(self.dlength)
        self.luminance_batch    = np.zeros(self.dlength)
        # preallocate outputs:
        self.out_i              = 0
        self.speeds             = np.empty(self.out_len, dtype=self.dtype)
        self.timestamps         = np.empty(self.out_len, dtype=self.dtype)
        self.points_2d          = np.empty((self.out_len * self.dlength, 3), dtype=self.dtype)  # [[x, y, l],[..
        
        # file format and visualization
        self.data_dir           = data_dir
        self.format             = format
        self.visualization      = visualization

        self.platform          = get_platform()

        if self.platform == 'MCU':
            pwm_pin = "GP2"
            pwm_dc = 0.4
            pwm_dc_16bit = int(pwm_dc * 65534)
            pwm = init_pwm(pwm_pin)
            pwm.duty_cycle = pwm_dc_16bit
    

    def close(self):
        print("Closing...")
        if self.visualization is not None:
            self.visualization.close()
        self.serial_connection.close()
        print("Serial connection closed.")
    

    def read_loop(self):
        while self.serial_connection.is_open and keyboard.is_pressed('q') is False:
            try:
                if self.out_i == self.out_len:
                    # SAVE DATA
                    if self.format is not None:
                        save_data(self.data_dir, self.points_2d, self.format)
                    # VISUALIZE
                    if self.visualization is not None:
                        self.visualization.update_coordinates(self.points_2d)
                    self.out_i = 0
                self.read()
            except serial.SerialException:
                print("Serial connection closed.")
                break
            self.out_i += 1


    def read(self):
        # iterate through serial stream until start package is found
        while self.serial_connection.is_open:
            data_byte = self.serial_connection.read()

            if data_byte == self.start_byte:
                self.byte_array.extend(data_byte)
                self.flag_2c = True
                continue

            elif data_byte == self.dlength_byte and self.flag_2c:
                self.byte_array.extend(data_byte)

                # Error handling
                if len(self.byte_array) != self.package_len:
                    # print("[WARNING] Incomplete:", self.byte_array)
                    self.byte_array = bytearray()
                    self.flag_2c = False
                    continue
                
                self.decode(self.byte_array)  # updates speed, timestamp, angle_batch, distance_batch, luminance_batch
                x_batch, y_batch = self.polar2cartesian(self.angle_batch, self.distance_batch, self.offset)
                points_batch = np.column_stack((x_batch, y_batch, self.luminance_batch)).astype(self.dtype)

                # write into preallocated output arrays at current index
                self.speeds[self.out_i] = self.speed
                self.timestamps[self.out_i] = self.timestamp
                self.points_2d[self.out_i*self.dlength:(self.out_i+1)*self.dlength] = points_batch

                # reset byte_array
                self.byte_array = bytearray()
                break

            else:
                self.byte_array.extend(data_byte)

            self.flag_2c = False


    def decode(self, byte_array):  
        # dlength = 12  # byte_array[46] & 0x1F
        self.speed = int.from_bytes(byte_array[0:2][::-1], 'big') / 360         # rotational frequency in rps
        FSA = float(int.from_bytes(byte_array[2:4][::-1], 'big')) / 100         # start angle in degrees
        LSA = float(int.from_bytes(byte_array[40:42][::-1], 'big')) / 100       # end angle in degrees
        self.timestamp = int.from_bytes(byte_array[42:44][::-1], 'big')         # timestamp in milliseconds < 30000
        CS = int.from_bytes(byte_array[44:45][::-1], 'big')                     # CRC Checksum                                
        
        angleStep = ((LSA - FSA) if LSA - FSA > 0 else (LSA + 360 - FSA)) / (self.dlength-1)

        # 3 bytes per sample x 12 samples
        for counter, i in enumerate(range(0, 3 * self.dlength, 3)): 
            self.angle_batch[counter] = ((angleStep * counter + FSA) % 360) * self.deg2rad
            self.distance_batch[counter] = int.from_bytes(byte_array[4 + i:6 + i][::-1], 'big') / 100
            self.luminance_batch[counter] = byte_array[6 + i]


    @staticmethod
    def polar2cartesian(angles, distances, offset):
        angles = list(np.array(angles) + offset)
        x_list = distances * -np.cos(angles)
        y_list = distances * np.sin(angles)
        return x_list, y_list



if __name__ == "__main__":
    import os
    from matplotlib_utils import plot_2D

    # CONSTANTS
    PORT = 'COM1'  # {'Windows': 'COM10', 'RaspberryPi': '/dev/ttyACM0', 'Linux': '/dev/ttyUSB0'}[platform.system()] 
    ANGLE_OFFSET = np.pi / 2    # = 90°
    FORMAT = 'npy'              # 'npy' or 'csv' or None
    DTYPE = np.float32
    DATA_DIR = "cpython/data"
    VISUALIZATION = plot_2D()   # plot_2D() or None
    OUT_LEN = 40                # visualize after every nth batch

    # ensure output directory
    if not os.path.exists(DATA_DIR):
        os.makedirs(DATA_DIR)

    lidar = LD06(port=PORT,
                visualization=VISUALIZATION,
                offset=ANGLE_OFFSET, 
                format=FORMAT,
                dtype=DTYPE,
                data_dir=DATA_DIR,
                out_len=OUT_LEN)

    try:
        if lidar.serial_connection.is_open:
            lidar.read_loop()
    finally:
        lidar.close()
