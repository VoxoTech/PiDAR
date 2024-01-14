import numpy as np
import math
import os
from lib.matplotlib_utils import plot_2D
from lib.LD06_driver import LD06


# CONSTANTS
PORT = "COM10"  # {'Windows': 'COM10', 'RaspberryPi': '/dev/ttyACM0', 'Linux': '/dev/ttyUSB0'}[platform.system()] 
ANGLE_OFFSET = math.pi / 2  # 90°
SAVE = 'npy'  # was 'csv' before
DTYPE = np.float32
DATA_DIR = "cpython/data"
VIZ_INTERVAL = 40  # visualize after every nth batch

# ensure output directory
if not os.path.exists(DATA_DIR):
    os.makedirs(DATA_DIR)

lidar = LD06(port=PORT,
            visualization=plot_2D(),
            offset=ANGLE_OFFSET, 
            save=SAVE,
            dtype=DTYPE,
            data_dir=DATA_DIR,
            viz_interval=VIZ_INTERVAL)

try:
    if lidar.serial_connection.is_open:
        lidar.read_continuously()
finally:
    lidar.close()
