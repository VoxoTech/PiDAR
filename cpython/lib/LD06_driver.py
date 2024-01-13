'''
LD06 datasheet:
https://storage.googleapis.com/mauser-public-images/prod_description_document/2021/315/8fcea7f5d479f4f4b71316d80b77ff45_096-6212_a.pdf

375 batches/s x 12 samples/batch = 4500 samples/s
'''

import math
import numpy as np
import threading
import time
import csv
import serial
import keyboard


class LD06:
    def __init__(self, port, offset=0, data_dir="data", save_csv=False, delimiter=",", visualization=None, viz_interval=40):
        self.port = port
        self.serial_connection = LD06_serial(self.port)
        self.offset = offset

        self.data_dir = data_dir
        self.save_csv = save_csv
        self.delimiter = delimiter
        self.visualization = visualization
        self.viz_interval = viz_interval

        self.byte_string = ""
        self.flag_2c = False
        self.viz_counter = 0

        self.speeds = list()  
        self.timestamps = list()
        self.points_2d = np.empty((0,3), dtype=np.float32)  # np.array([[luminance, x, y], ...])
    

    def close(self):
        print("Closing...")
        if self.visualization is not None:
            self.visualization.close()

        self.serial_connection.close()
        print("Serial connection closed.")
    

    def read_continuously(self):
        while self.serial_connection.is_open and keyboard.is_pressed('q') is False:
            self.viz_counter += 1

            try:
                if self.viz_counter == self.viz_interval:
                    if self.save_csv:
                        t = threading.Thread(target=save_from_coordinates, args=(self.data_dir, self.points_2d, self.delimiter))
                        t.start()
                    
                    if self.visualization is not None:
                        self.visualization.update_from_coordinates(self.points_2d)

                    # reset lists
                    self.speeds.clear()
                    self.timestamps.clear()
                    self.points_2d = np.empty((0,3), dtype=np.float32)
                    self.viz_counter = 0
            
                # iterate through serial stream until start or end of package is found
                flag_2c = False
                while self.serial_connection.is_open:

                    data_byte = self.serial_connection.read()
                    data_int = int.from_bytes(data_byte, 'big')

                    if data_int == 0x54:  # start of package
                        self.byte_string += data_byte.hex() + " "
                        flag_2c = True
                        continue

                    elif data_int == 0x2c and flag_2c: # end of package
                        self.byte_string += data_byte.hex()

                        # if len(self.byte_string[0:-6].replace(' ', '')) != 90:
                        if len(self.byte_string) != 140:  # 90 byte + spaces + 0x54 + 0x2c
                            self.byte_string = ""
                            flag_2c = False
                            continue
                        
                        ## crop last two byte (0x54, 0x2c) from byte_string
                        speed, timestamp, angle_batch, distance_batch, luminance_batch = self.decode_string(self.byte_string[0:-5])
                        x_batch, y_batch = polar2cartesian(angle_batch, distance_batch, self.offset)

                        self.speeds.append(speed)
                        self.timestamps.append(timestamp)
                        self.points_2d = np.vstack((self.points_2d, np.column_stack((x_batch, y_batch, luminance_batch))))
                        self.byte_string = ""
                        break

                    else:
                        self.byte_string += data_byte.hex() + " "

                    flag_2c = False

            except serial.SerialException:
                print("Serial connection closed.")
                break

    
    @staticmethod
    def decode_string(string):
        string = string.replace(' ', '')        
        
        dlength = 12  # int(string[2:4], 16) & 0x1F 
        speed = int(string[2:4] + string[0:2], 16) / 100            # rotational speed in degrees/second
        FSA = float(int(string[6:8] + string[4:6], 16)) / 100       # start angle in degrees
        LSA = float(int(string[-8:-6] + string[-10:-8], 16)) / 100  # end angle in degrees
        timestamp = int(string[-4:-2] + string[-6:-4], 16)          # timestamp in milliseconds
        CS = int(string[-2:], 16)                                   # CRC Checksum                                
        
        angleStep = (LSA - FSA) / (dlength-1) if LSA - FSA > 0 else (LSA + 360 - FSA) / (dlength-1)
        
        angle_batch = list()
        distance_batch = list()
        luminance_batch = list()

        counter = 0
        circle = lambda deg: deg - 360 if deg >= 360 else deg
        for i in range(0, 6 * 12, 6):

            angle = circle(angleStep * counter + FSA) * math.pi / 180.0
            angle_batch.append(angle)

            distance = int(string[8 + i + 2:8 + i + 4] + string[8 + i:8 + i + 2], 16) / 100
            distance_batch.append(distance)

            luminance = int(string[8 + i + 4:8 + i + 6], 16)
            luminance_batch.append(luminance)

            counter += 1

        return speed, timestamp, angle_batch, distance_batch, luminance_batch


def LD06_serial(port):
    # import platform
    # port = {'Windows': 'COM10', 'RaspberryPi': '/dev/ttyACM0', 'Linux': '/dev/ttyUSB0'}[platform.system()]  
    return serial.Serial(port=port, baudrate=230400, timeout=1.0, bytesize=8, parity='N', stopbits=1)


def polar2cartesian(angles, distances, offset):
    # angles = list(np.array(angles) + offset)
    # x_list = distances * -np.cos(angles)
    # y_list = distances * np.sin(angles)

    angles = [a + offset for a in angles]
    x_list = [d * -math.cos(a) for a, d in zip(angles, distances)]
    y_list = [d * math.sin(a) for a, d in zip(angles, distances)]

    return x_list, y_list


def save_from_coordinates(save_dir, points_2d, delimiter):
    filename = f"{save_dir}/{time.time()}.csv"
    
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f, delimiter=delimiter)
        writer.writerows(points_2d)



if __name__ == "__main__":
    import math
    import os
    from matplotlib_utils import plot_2D

    # CONSTANTS
    PORT = "COM10"
    ANGLE_OFFSET = math.pi / 2  # 90°
    SAVE_CSV = True
    DATA_DIR = "cpython/data"
    DELIMITER = ","
    VIZ_INTERVAL = 40  # visualize after every nth batch

    # ensure output directory
    if not os.path.exists(DATA_DIR):
        os.makedirs(DATA_DIR)

    lidar = LD06(port=PORT,
                visualization=plot_2D(),
                offset=ANGLE_OFFSET, 
                save_csv=SAVE_CSV,
                data_dir=DATA_DIR,
                delimiter=DELIMITER,
                viz_interval=VIZ_INTERVAL)

    try:
        if lidar.serial_connection.is_open:
            lidar.read_continuously()
    finally:
        lidar.close()
