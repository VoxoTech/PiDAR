import math
import os
from lib.matplotlib_utils import plot_2D
from lib.LD06_driver import LD06

# CONSTANTS
PORT = "COM10" # port = {'Windows': 'COM10', 'RaspberryPi': '/dev/ttyACM0', 'Linux': '/dev/ttyUSB0'}[platform.system()]  
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
