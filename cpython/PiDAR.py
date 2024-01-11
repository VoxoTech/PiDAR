import math
import os
import threading
import keyboard

from lib.LD06_driver import LD06_serial, decode_string
from lib.file_utils import save_thread
from lib.matplotlib_utils import plot_2D


# PARAMETERS
port = "COM10"
angle_offset = math.pi / 2  # 90°
save_csv = True
data_dir = "cpython/data"
delimiter = ";"
update_interval = 40


# create output directory
if not os.path.exists(data_dir):
    os.makedirs(data_dir)

# init output lists
x_list = list()
y_list = list()
luminance_list = list()

visualisation = plot_2D()

# SERIAL CONNECTION
with LD06_serial(port) as serial_connection:
    byte_string = ""
    i = 0
    while keyboard.is_pressed('q') is False:

        # save data to csv, visualise data and clear lists
        if i % update_interval == update_interval - 1:
            visualisation.update_data(x_list, y_list, luminance_list)

            if save_csv:
                t = threading.Thread(target=save_thread, args=(data_dir, x_list, y_list, luminance_list, delimiter))
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
                
                ## crop last two byte (0x54, 0x2c) from byte_string
                lidar_data = decode_string(byte_string[0:-5], angle_offset)

                x_list.extend(lidar_data.x)
                y_list.extend(lidar_data.y)
                luminance_list.extend(lidar_data.luminance_list)

                byte_string = ""
                break

            else:
                byte_string += data_byte.hex() + " "

            flag_2c = False
        i += 1
