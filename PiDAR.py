'''
LD06 LiDAR data processing and visualisation
'''

import math
import numpy as np
import os
import threading
import keyboard

from lib.LD06_serial import LD06_serial
from lib.utils_csv import save_thread

from lib.matplotlib_utils import plot_2D  # plot_3D, rotate_3D
from lib.open3d_utils import plot_3D, rotate_3D


class LD06_data:
    def __init__(self, FSA, LSA, CS, speed, timeStamp, luminance_list, angle_list, distance_list, offset):
        # raw sensor data (polar coordinates):
        self.FSA = FSA
        self.LSA = LSA
        self.CS = CS
        self.speed = speed
        self.timeStamp = timeStamp
        self.luminance_list = luminance_list
        self.angle_list = angle_list
        self.distance_list = distance_list
        self.offset = offset
        
        # compute cartesian coordinates
        self.x, self.y = self.polar2cartesian(self.angle_list, self.distance_list, self.offset)
    
    # https://stackoverflow.com/questions/20924085/python-conversion-between-coordinates
    @staticmethod
    def polar2cartesian(angle, distance, offset):
        angle = list(np.array(angle) + offset)
        x = distance * -np.cos(angle)
        y = distance * np.sin(angle)
        return x, y
    
    # @staticmethod
    # def cartesian2polar(x, y):
        distance = np.sqrt(x**2 + y**2)
        angle = np.arctan2(y, x)
        return angle, distance

    @staticmethod
    def compute(string, offset=0):
        string = string.replace(' ', '')

        # convert hex string at indices 2 and 3 to an integer and perform bitwise AND to get last 5 bits.
        # TODO: protocol returns 14 altough 12 is definitely correct. why?
        dlength = 12 # int(string[2:4], 16) & 0x1F  

        speed = int(string[2:4] + string[0:2], 16) / 100            # rotational speed in degrees/second
        FSA = float(int(string[6:8] + string[4:6], 16)) / 100       # start angle in degrees
        LSA = float(int(string[-8:-6] + string[-10:-8], 16)) / 100  # end angle in degrees
        timestamp = int(string[-4:-2] + string[-6:-4], 16)          # timestamp in milliseconds
        CS = int(string[-2:], 16)                                   # CRC Checksum                                
        

        angleStep = (LSA - FSA) / (dlength-1) if LSA - FSA > 0 else (LSA + 360 - FSA) / (dlength-1)
        
        angle_list = list()
        distance_list = list()
        luminance_list = list()

        counter = 0
        circle = lambda deg: deg - 360 if deg >= 360 else deg
        for i in range(0, 6 * 12, 6):

            angle = circle(angleStep * counter + FSA) * math.pi / 180.0
            angle_list.append(angle)

            distance = int(string[8 + i + 2:8 + i + 4] + string[8 + i:8 + i + 2], 16) / 100
            distance_list.append(distance)

            luminance = int(string[8 + i + 4:8 + i + 6], 16)
            luminance_list.append(luminance)

            counter += 1

        return LD06_data(FSA, LSA, CS, speed, timestamp, luminance_list, angle_list, distance_list, offset)
    

    # TODO: try using bytearray() instead of string
    # https://stackoverflow.com/questions/7555689/python-3-building-an-array-of-bytes
    @staticmethod
    def compute_bytes(data_bytes, offset=0):
        dlength = 12 

        # rotational speed in degrees/second
        speed = int.from_bytes(data_bytes[2:4], 'big') / 100
        
        # start angle in degrees
        FSA = float(int.from_bytes(data_bytes[4:6], 'big')) / 100

        ## INFO: if it was a actual float, convert 4 bytes to float
        # FSA = struct.unpack('f', byte_data[2:6])[0]
        
        # end angle in degrees
        LSA = float(int.from_bytes(data_bytes[-6:-4], 'big')) / 100  
        
        # timestamp in milliseconds
        timestamp = int.from_bytes(data_bytes[-4:-2], 'big')          
        
        # CRC Checksum
        CS = data_bytes[-1]
        

        angleStep = (LSA - FSA) / (dlength-1) if LSA - FSA > 0 else (LSA + 360 - FSA) / (dlength-1)
        
        angle_list = list()
        distance_list = list()
        luminance_list = list()

        counter = 0
        circle = lambda deg: deg - 360 if deg >= 360 else deg
        for i in range(0, 3 * dlength, 3):

            angle = circle(angleStep * counter + FSA) * math.pi / 180.0
            angle_list.append(angle)

            distance = int.from_bytes(data_bytes[6 + i:8 + i], 'big') / 100
            distance_list.append(distance)

            luminance = data_bytes[8 + i]
            luminance_list.append(luminance)

            counter += 1
        
        return LD06_data(FSA, LSA, CS, speed, timestamp, luminance_list, angle_list, distance_list, offset)


# PARAMETERS
angle_offset = math.pi / 2  # 90°
visualize = True 
save_csv = False
csv_dir = "csv"
update_interval = 40


# create output directory
if not os.path.exists(csv_dir):
    os.makedirs(csv_dir)

# init output lists
x_list = list()
y_list = list()
luminance_list = list()

if visualize:
    visualisation = plot_2D()

# SERIAL CONNECTION
with LD06_serial() as serial_connection:
    byte_string = ""
    #data_bytes = b""
    i = 0
    while True:
        # quit by pressing 'q'
        if keyboard.is_pressed('q'):
            break

        # save data to csv, visualise data and clear lists
        if i % update_interval == update_interval - 1:
            if visualize:
                visualisation.update_data(x_list, y_list, luminance_list)

            if save_csv:
                t = threading.Thread(target=save_thread, args=(csv_dir, x_list, y_list, luminance_list))
                t.start()

            # prepare for next iteration
            x_list.clear()
            y_list.clear()
            luminance_list.clear()
            i = 0

        # iterate through serial stream until start or end of package is found
        flag_2c = False
        while True:
            data_byte = serial_connection.read()
            data_int = int.from_bytes(data_byte, 'big')

            if data_int == 0x54:  # start of package
                byte_string += data_byte.hex() + " "
                flag_2c = True
                continue

            elif data_int == 0x2c and flag_2c: # end of package
                byte_string += data_byte.hex()

                # if len(byte_string[0:-6].replace(' ', '')) != 90:
                if len(byte_string) != 140:  # 90 byte + spaces + 0x54 + 0x2c
                    byte_string = ""
                    flag_2c = False
                    continue
                
                # crop last two byte (0x54, 0x2c) from byte_string
                lidar_data = LD06_data.compute(byte_string[0:-5], angle_offset)
                # lidar_data = LD06_data.compute_bytes(byte_array[0:-5], angle_offset)

                x_list.extend(lidar_data.x)
                y_list.extend(lidar_data.y)
                luminance_list.extend(lidar_data.luminance_list)

                byte_string = ""
                break

            else:
                byte_string += data_byte.hex() + " "

            flag_2c = False

        i += 1



# ------------------
# 3D CONVERSION AND VISUALISATION

def listfiles(dir, extension=None):
    filepaths = list()
    for root, dirs, files in os.walk(dir, topdown=False):
        for file in files:
            if extension is None or os.path.splitext(file)[1] == extension:
                filepath = os.path.join(root, file)
                filepaths.append(filepath)
    return filepaths


if __name__ == "__main__" and save_csv:
    # 3D conversion
    rotation_axis = np.array([0, 0, 1])  # 3D rotation about up-axis: Z
    angular_resolution = 1  # in degrees

    # init result object with (X,y,Z, intensity)
    pointcloud = np.zeros((1, 4))
    angle = 0

    filepaths = listfiles(csv_dir, extension=".csv")

    for filepath in filepaths:
        points2d = np.loadtxt(filepath, delimiter=";")

        # insert 3D Y=0 after column 0 so 2D Y becomes 3D Z (Z-up: image is now vertical)
        points3d = np.insert(points2d, 1, values=0, axis=1)

        points3d = rotate_3D(points3d, rotation_axis, angle)
        pointcloud = np.append(pointcloud, points3d, axis=0)

        angle += angular_resolution

    plot_3D(pointcloud)
