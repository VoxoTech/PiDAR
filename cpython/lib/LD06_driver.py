'''
LD06 datasheet:
https://storage.googleapis.com/mauser-public-images/prod_description_document/2021/315/8fcea7f5d479f4f4b71316d80b77ff45_096-6212_a.pdf

375 batches/s x 12 samples/batch = 4500 samples/s
'''

import numpy as np
import threading
import time
import serial
import keyboard
import csv


class LD06:
    def __init__(self, port, offset=0, data_dir="data", save=None, visualization=None, viz_interval=40, dtype=np.float32):
        self.port = port
        self.serial_connection = LD06_serial(self.port)
        self.offset = offset

        self.data_dir = data_dir
        self.save = save
        self.visualization = visualization
        self.viz_interval = viz_interval

        self.byte_array = bytearray()
        self.flag_2c = False
        self.viz_counter = 0

        self.speeds = list()  
        self.timestamps = list()
        self.dtype = dtype
        self.points_2d = np.empty((0,3), dtype=self.dtype)
    
    def close(self):
        print("Closing...")
        if self.visualization is not None:
            self.visualization.close()

        self.serial_connection.close()
        print("Serial connection closed.")
    
    def read_loop(self):
        while self.serial_connection.is_open and keyboard.is_pressed('q') is False:
            self.viz_counter += 1

            try:
                if self.viz_counter == self.viz_interval:
                    # SAVE DATA
                    if self.save == 'npy':
                        t = threading.Thread(target=save_npy, args=(self.data_dir, self.points_2d))
                        t.start()
                    if self.save == 'csv':
                        t = threading.Thread(target=save_csv, args=(self.data_dir, self.points_2d))
                        t.start()
                    
                    # VISUALIZE
                    if self.visualization is not None:
                        self.visualization.update_coordinates(self.points_2d)

                    # reset lists
                    self.speeds.clear()
                    self.timestamps.clear()
                    self.points_2d = np.empty((0,3), dtype=self.dtype)
                    self.viz_counter = 0
            
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
                        
                        speed, timestamp, angle_batch, distance_batch, luminance_batch = self.decode(self.byte_array)
                        
                        x_batch, y_batch = polar2cartesian(angle_batch, distance_batch, self.offset)

                        self.speeds.append(speed)
                        self.timestamps.append(timestamp)

                        # append to points_2d
                        self.points_2d = np.vstack((self.points_2d, np.column_stack((x_batch, y_batch, luminance_batch)))).astype(self.dtype)

                        # reset byte_array
                        self.byte_array = bytearray()
                        break

                    else:
                        self.byte_array.extend(data_byte)

                    flag_2c = False

            except serial.SerialException:
                print("Serial connection closed.")
                break

    @staticmethod
    def decode(byte_array):  
        dlength = 12  # byte_array[46] & 0x1F
        speed = int.from_bytes(byte_array[0:2][::-1], 'big') / 100            # rotational speed in degrees/second
        FSA = float(int.from_bytes(byte_array[2:4][::-1], 'big')) / 100       # start angle in degrees
        LSA = float(int.from_bytes(byte_array[40:42][::-1], 'big')) / 100     # end angle in degrees
        timestamp = int.from_bytes(byte_array[42:44][::-1], 'big')            # timestamp in milliseconds
        CS = int.from_bytes(byte_array[44:45][::-1], 'big')                   # CRC Checksum                                

        angleStep = (LSA - FSA) / (dlength-1) if LSA - FSA > 0 else (LSA + 360 - FSA) / (dlength-1)

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


def LD06_serial(port):
    # import platform
    # port = {'Windows': 'COM10', 'RaspberryPi': '/dev/ttyACM0', 'Linux': '/dev/ttyUSB0'}[platform.system()]  
    return serial.Serial(port=port, baudrate=230400, timeout=1.0, bytesize=8, parity='N', stopbits=1)

def polar2cartesian(angles, distances, offset):
    radians = list(np.array(angles) + offset)
    x_list = distances * -np.cos(radians)
    y_list = distances * np.sin(radians)
    return x_list, y_list

def save_csv(save_dir, points_2d, delimiter=','):
    filename = f"{save_dir}/{time.time()}.csv"
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f, delimiter=delimiter)
        writer.writerows(points_2d)

def save_npy(save_dir, points_2d):
    filename = f"{save_dir}/{time.time()}.npy"
    np.save(filename, points_2d)


if __name__ == "__main__":
    import os
    from matplotlib_utils import plot_2D

    # CONSTANTS
    PORT = "COM10"  # {'Windows': 'COM10', 'RaspberryPi': '/dev/ttyACM0', 'Linux': '/dev/ttyUSB0'}[platform.system()] 
    ANGLE_OFFSET = np.pi / 2  # = 90°
    SAVE = 'npy'                # 'npy' or 'csv'
    DTYPE = np.float32
    DATA_DIR = "cpython/data"
    VISUALIZATION = plot_2D()   # plot_2D() or None
    VIZ_INTERVAL = 40           # visualize after every nth batch

    # ensure output directory
    if not os.path.exists(DATA_DIR):
        os.makedirs(DATA_DIR)

    lidar = LD06(port=PORT,
                visualization=VISUALIZATION,
                offset=ANGLE_OFFSET, 
                save=SAVE,
                dtype=DTYPE,
                data_dir=DATA_DIR,
                viz_interval=VIZ_INTERVAL)

    try:
        if lidar.serial_connection.is_open:
            lidar.read_loop()
    finally:
        lidar.close()
