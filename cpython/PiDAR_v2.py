import serial
import math
import threading
import csv
import time
# import platform

from lib.matplotlib_utils import plot_2D
from lib.file_utils import save_thread


class LD06:
    def __init__(self, port):
        self.port = port
        self.serial_connection = serial.Serial(port=port, baudrate=230400, timeout=5.0, bytesize=8, parity='N', stopbits=1)
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
                speed, FSA, LSA, timestamp, CS, angle_batch, distance_batch, luminance_batch = self.decode(data_package)  #[1:]

                # Calculate cartesian coordinates from angles and distances
                x_batch, y_batch = self.polar2cartesian(angle_batch, distance_batch, self.angle_offset)
                return x_batch, y_batch, luminance_batch
            
            else:
                data_package += bytearray(single_byte)

            flag_2c = False
    

    @staticmethod
    def decode(data_package):
        # Extract the fields from the data_package according to the protocol
        speed = int.from_bytes(data_package[0:2], 'big') / 100
        FSA = int.from_bytes(data_package[2:4], 'big') / 100
        data = int.from_bytes(data_package[4:40], 'big') / 100
        LSA = int.from_bytes(data_package[40:42], 'big') / 100
        timestamp = int.from_bytes(data_package[42:44], 'big') % 3000
        CS = int.from_bytes(data_package[44:45], 'big')
        
        angleStep = (LSA - FSA) / 11 if LSA - FSA > 0 else (LSA + 360 - FSA) / 11
        
        angle_batch = list()
        distance_batch = list()
        luminance_batch = list()

        for i in range(0, 12):
            angle = (angleStep * i + FSA) % 360
            angle_batch.append(angle)

            distance = int.from_bytes(data_package[4 + i*3:6 + i*3], 'big') / 100
            distance_batch.append(distance)

            luminance = int.from_bytes(data_package[6 + i*3:7 + i*3], 'big')
            luminance_batch.append(luminance)
        
        return speed, FSA, LSA, timestamp, CS, angle_batch, distance_batch, luminance_batch


    @staticmethod
    def polar2cartesian(angles, distances, offset):
        angles = [math.radians(a + offset) for a in angles]
        x_list = [d * -math.cos(a) for a, d in zip(angles, distances)]
        y_list = [d * math.sin(a) for a, d in zip(angles, distances)]
        return x_list, y_list
    
    
    def close(self):
       self.serial_connection.close()



if __name__ == "__main__":
    
    data_dir = "cpython/data"
    output_len = 4500
    save_csv = True
    csv_delimiter = ";"

    # port = {'Windows': 'COM10', 'RaspberryPi': '/dev/ttyACM0', 'Linux': '/dev/ttyUSB0'}[platform.system()]  
    lidar = LD06('COM10')

    visualisation = plot_2D()

    # inits
    x_list = []
    y_list = []
    luminance_list = []
    counter = 0
    
    try:
        while True:
            x_batch, y_batch, luminance_batch = lidar.read()

            x_list += x_batch
            y_list += y_batch
            luminance_list += luminance_batch
            counter += len(x_batch)

            if counter >= output_len:
                visualisation.update_data(x_list, y_list, luminance_list)

                if save_csv:
                    t = threading.Thread(target=save_thread, args=(data_dir, x_list, y_list, luminance_list, csv_delimiter))
                    t.start()
            
                x_list.clear()
                y_list.clear()
                luminance_list.clear()
                counter = 0
    finally:
        lidar.close()
        print("LIDAR closed")
