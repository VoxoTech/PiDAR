'''
LD06 LiDAR data processing and visualisation
'''
import platform
import serial
import math
import numpy as np
import os
import threading
import keyboard

from lib.file_utils import save_thread, list_files, merge_csv_dir
from lib.matplotlib_utils import plot_2D  # plot_3D, rotate_3D
from lib.open3d_utils import plot_3D, rotate_3D


def LD06_serial():
    '''
    specification of LD06 dataframe:
    https://storage.googleapis.com/mauser-public-images/prod_description_document/2021/315/8fcea7f5d479f4f4b71316d80b77ff45_096-6212_a.pdf
    '''
    port = {'Windows': 'COM10', 
            'Linux': '/dev/ttyUSB0'}[platform.system()]  # 'Raspberry': '/dev/ttyACM0'
    return serial.Serial(port=port, baudrate=230400, timeout=5.0, bytesize=8, parity='N', stopbits=1)


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
csv_dir = "cpython/csv"
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
                print(byte_string[0:-5])
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



# 3D CONVERSION AND VISUALISATION
if save_csv:
    pointcloud = merge_csv_dir(csv_dir, angle_step=1)
    plot_3D(pointcloud)
