'''
LD06 datasheet:
https://storage.googleapis.com/mauser-public-images/prod_description_document/2021/315/8fcea7f5d479f4f4b71316d80b77ff45_096-6212_a.pdf

375 batches/s x 12 samples/batch = 4500 samples/s
'''

import numpy as np
import threading
import serial
import keyboard

try:
    from lib.file_utils import save_npy, save_csv
except:
    from file_utils import save_npy, save_csv
    

class LD06:
    def __init__(self, port, offset=0, data_dir="data", format=None, visualization=None, interval=40, dtype=np.float32):
        # angle offset
        self.offset = offset

        # serial
        self.port = port
        self.serial_connection = self.init_serial(self.port)
        self.byte_array = bytearray()
        self.dlength = 12
        self.flag_2c = False
        self.dtype = dtype
        
        # init ouput array
        self.interval = interval
        self.__init_arrays__()
        
        # file format and visualization
        self.data_dir = data_dir
        self.format = format
        self.visualization = visualization


    def __init_arrays__(self):
        self.points_2d  = np.empty((self.interval * self.dlength, 3), dtype=self.dtype)
        self.speeds     = np.empty(self.interval, dtype=self.dtype)
        self.timestamps = np.empty(self.interval, dtype=self.dtype)
        self.counter = 0


    @staticmethod
    def init_serial(port):
        # import platform
        # port = {'Windows': 'COM10', 'RaspberryPi': '/dev/ttyACM0', 'Linux': '/dev/ttyUSB0'}[platform.system()]  
        return serial.Serial(port=port, baudrate=230400, timeout=1.0, bytesize=8, parity='N', stopbits=1)
    

    def close(self):
        print("Closing...")
        if self.visualization is not None:
            self.visualization.close()

        self.serial_connection.close()
        print("Serial connection closed.")
    

    def read_loop(self):
        while self.serial_connection.is_open and keyboard.is_pressed('q') is False:
            try:
                if self.counter == self.interval:
                    # SAVE DATA
                    if self.format == 'npy':
                        t = threading.Thread(target=save_npy, args=(self.data_dir, self.points_2d))
                        t.start()
                    if self.format == 'csv':
                        t = threading.Thread(target=save_csv, args=(self.data_dir, self.points_2d))
                        t.start()
                    
                    # VISUALIZE
                    if self.visualization is not None:
                        self.visualization.update_coordinates(self.points_2d)

                    # reset lists
                    self.__init_arrays__()
            
                # iterate through serial stream until start package is found
                flag_2c = False
                while self.serial_connection.is_open:

                    data_byte = self.serial_connection.read()

                    if data_byte == bytes([0x54]):
                        self.byte_array.extend(data_byte)
                        flag_2c = True
                        continue

                    elif data_byte == bytes([0x2c]) and flag_2c:
                        self.byte_array.extend(data_byte)

                        if len(self.byte_array) != 47:  # 45 byte + 0x54 + 0x2c
                            self.byte_array = bytearray()
                            flag_2c = False
                            continue
                        
                        speed, timestamp, angle_batch, distance_batch, luminance_batch = self.decode(self.byte_array, dlength=self.dlength)

                        x_batch, y_batch = self.polar2cartesian(angle_batch, distance_batch, self.offset)
                        points_batch = np.column_stack((x_batch, y_batch, luminance_batch)).astype(self.dtype)

                        # append to output arrays
                        self.speeds[self.counter] = speed
                        self.timestamps[self.counter] = timestamp
                        self.points_2d[self.counter*self.dlength:(self.counter+1)*self.dlength] = points_batch

                        # reset byte_array
                        self.byte_array = bytearray()
                        break

                    else:
                        self.byte_array.extend(data_byte)

                    flag_2c = False

            except serial.SerialException:
                print("Serial connection closed.")
                break

            self.counter += 1


    @staticmethod
    def decode(byte_array, dlength=12):  
        # dlength = 12  # byte_array[46] & 0x1F
        speed = int.from_bytes(byte_array[0:2][::-1], 'big') / 100            # rotational speed in degrees/second
        FSA = float(int.from_bytes(byte_array[2:4][::-1], 'big')) / 100       # start angle in degrees
        LSA = float(int.from_bytes(byte_array[40:42][::-1], 'big')) / 100     # end angle in degrees
        timestamp = int.from_bytes(byte_array[42:44][::-1], 'big')            # timestamp in milliseconds
        CS = int.from_bytes(byte_array[44:45][::-1], 'big')                   # CRC Checksum                                

        angleStep = ((LSA - FSA) if LSA - FSA > 0 else (LSA + 360 - FSA)) / (dlength-1)

        angle_batch = list()
        distance_batch = list()
        luminance_batch = list()

        # 3 bytes per sample x 12 samples
        for counter, i in enumerate(range(0, 3 * dlength, 3)): 
            angle = ((angleStep * counter + FSA) % 360) * np.pi / 180.0
            angle_batch.append(angle)

            distance = int.from_bytes(byte_array[4 + i:6 + i][::-1], 'big') / 100
            distance_batch.append(distance)

            luminance = byte_array[6 + i]
            luminance_batch.append(luminance)

        return speed, timestamp, angle_batch, distance_batch, luminance_batch


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
    PORT = "COM10"  # {'Windows': 'COM10', 'RaspberryPi': '/dev/ttyACM0', 'Linux': '/dev/ttyUSB0'}[platform.system()] 
    ANGLE_OFFSET = np.pi / 2    # = 90°
    FORMAT = 'csv'              # 'npy' or 'csv'
    DTYPE = np.float32
    DATA_DIR = "cpython/data"
    VISUALIZATION = plot_2D()   # plot_2D() or None
    INTERVAL = 40               # visualize after every nth batch

    # ensure output directory
    if not os.path.exists(DATA_DIR):
        os.makedirs(DATA_DIR)

    lidar = LD06(port=PORT,
                visualization=VISUALIZATION,
                offset=ANGLE_OFFSET, 
                format=FORMAT,
                dtype=DTYPE,
                data_dir=DATA_DIR,
                interval=INTERVAL)

    try:
        if lidar.serial_connection.is_open:
            lidar.read_loop()
    finally:
        lidar.close()
