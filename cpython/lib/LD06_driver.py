'''
LD06 datasheet:
https://storage.googleapis.com/mauser-public-images/prod_description_document/2021/315/8fcea7f5d479f4f4b71316d80b77ff45_096-6212_a.pdf
'''

import serial
import math
import numpy as np
# import platform


def LD06_serial(port):
    # port = {'Windows': 'COM10', 'Linux': '/dev/ttyUSB0', 'Raspberry': '/dev/ttyACM0'}
    return serial.Serial(port=port, baudrate=230400, timeout=5.0, bytesize=8, parity='N', stopbits=1)



class LD06:
    def __init__(self, FSA, LSA, CS, speed, timeStamp, luminance_list, angle_list, distance_list, offset):
        self.FSA = FSA
        self.LSA = LSA
        self.CS = CS
        self.speed = speed
        self.timeStamp = timeStamp
        self.luminance_list = luminance_list
        self.angle_list = angle_list
        self.distance_list = distance_list
        self.offset = offset
        self.x, self.y = polar2cartesian(self.angle_list, self.distance_list, self.offset, use_np=True)



class LD06_v2:
    def __init__(self, port):
        self.port = port
        self.serial_connection = LD06_serial(self.port)
        self.package_len = 48
        self.start_byte = b'T'  # 0x54 == 84
        self.len_byte = b','    # 0x2c == 44

        self.angle_offset = math.pi / 2 # 90°
        self.x_list = []
        self.y_list = []
        self.luminance_list = []
        self.csv_delimiter = ";"
    
    def read(self):
        # Detect the start of the package
        flag_2c = False
        data_package = bytearray()

        while True:
            single_byte = self.serial_connection.read()
            if single_byte == self.start_byte:  # if single_byte is not None and int.from_bytes(single_byte, 'big') == 0x54:  
                flag_2c = True
                data_package += bytearray(single_byte)
                continue

            elif single_byte == self.len_byte and flag_2c:
                data_package += bytearray(single_byte)

                if len(data_package) != self.package_len-1:
                    print("[WARNING] Incomplete data package")
                    data_package = bytearray()
                    flag_2c = False
                    continue
                    
                
                # # Read the entire package
                # print("Reading package")
                # data_package = self.serial_connection.read(self.package_len)

                # # Error handling
                # if data_package[0] != self.len_byte:
                #     # print("[WARNING] no Data Length Byte (0x2c) found")
                #     return [], [], []
                # # Check the length of data_package
                # elif len(data_package) < self.package_len:
                #     # print("[WARNING] Incomplete data package")
                #     return [], [], []

                # print(data_package, len(data_package))

                ## Drop first element (0x2c) of the bytearray and decode the remaining
                # TODO: those bytes are not in the package anymore ?!
                speed, FSA, LSA, timestamp, CS, angle_batch, distance_batch, luminance_batch = decode_bytes_v2(data_package)  #[1:]

                # Calculate cartesian coordinates from angles and distances
                x_batch, y_batch = polar2cartesian(angle_batch, distance_batch, self.angle_offset)
                return x_batch, y_batch, luminance_batch
            
            else:
                data_package += bytearray(single_byte)

            flag_2c = False

    def close(self):
       self.serial_connection.close()



def decode_string(string, offset=0):
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

    return LD06(FSA, LSA, CS, speed, timestamp, luminance_list, angle_list, distance_list, offset)
    


# # TODO: WORK-IN-PROGRESS
# def decode_bytes(data_bytes, offset=0):
#     dlength = 12 

#     # rotational speed in degrees/second
#     speed = int.from_bytes(data_bytes[2:4], 'big') / 100
    
#     # start angle in degrees
#     FSA = float(int.from_bytes(data_bytes[4:6], 'big')) / 100

#     ## INFO: if it was a actual float, convert 4 bytes to float
#     # FSA = struct.unpack('f', byte_data[2:6])[0]
    
#     # end angle in degrees
#     LSA = float(int.from_bytes(data_bytes[-6:-4], 'big')) / 100  
    
#     # timestamp in milliseconds
#     timestamp = int.from_bytes(data_bytes[-4:-2], 'big')          
    
#     # CRC Checksum
#     CS = data_bytes[-1]
    

#     angleStep = (LSA - FSA) / (dlength-1) if LSA - FSA > 0 else (LSA + 360 - FSA) / (dlength-1)
    
#     angle_list = list()
#     distance_list = list()
#     luminance_list = list()

#     counter = 0
#     circle = lambda deg: deg - 360 if deg >= 360 else deg
#     for i in range(0, 3 * dlength, 3):

#         angle = circle(angleStep * counter + FSA) * math.pi / 180.0
#         angle_list.append(angle)

#         distance = int.from_bytes(data_bytes[6 + i:8 + i], 'big') / 100
#         distance_list.append(distance)

#         luminance = data_bytes[8 + i]
#         luminance_list.append(luminance)

#         counter += 1
    
#     return FSA, LSA, CS, speed, timestamp, luminance_list, angle_list, distance_list, offset


def decode_bytes_v2(data_bytes):
    # Extract the fields from the data_package according to the protocol
    speed = int.from_bytes(data_bytes[0:2], 'big') / 100
    FSA = int.from_bytes(data_bytes[2:4], 'big') / 100
    data = int.from_bytes(data_bytes[4:40], 'big') / 100
    LSA = int.from_bytes(data_bytes[40:42], 'big') / 100
    timestamp = int.from_bytes(data_bytes[42:44], 'big') % 3000
    CS = int.from_bytes(data_bytes[44:45], 'big')
    
    angleStep = (LSA - FSA) / 11 if LSA - FSA > 0 else (LSA + 360 - FSA) / 11
    
    angle_batch = list()
    distance_batch = list()
    luminance_batch = list()

    for i in range(0, 12):
        angle = (angleStep * i + FSA) % 360
        angle_batch.append(angle)

        distance = int.from_bytes(data_bytes[4 + i*3:6 + i*3], 'big') / 100
        distance_batch.append(distance)

        luminance = int.from_bytes(data_bytes[6 + i*3:7 + i*3], 'big')
        luminance_batch.append(luminance)
    
    return speed, FSA, LSA, timestamp, CS, angle_batch, distance_batch, luminance_batch


# def cartesian2polar(x, y):
#     distance = np.sqrt(x**2 + y**2)
#     angle = np.arctan2(y, x)
#     return angle, distance


def polar2cartesian(angles, distances, offset, use_np=True, deg2rad=False):
    if use_np:
        angles = list(np.array(angles) + offset)
        x_list = distances * -np.cos(angles)
        y_list = distances * np.sin(angles)

    else:
        if deg2rad and not use_np:
            angles = [math.radians(a + offset) for a in angles]
        else:
            angles = [a + offset for a in angles]

        x_list = [d * -math.cos(a) for a, d in zip(angles, distances)]
        y_list = [d * math.sin(a) for a, d in zip(angles, distances)]

    return x_list, y_list
