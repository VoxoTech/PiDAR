import numpy as np
import os
from lib.matplotlib_utils import plot_2D
from lib.LD06_driver import LD06


# CONSTANTS
PORT = "COM10"  # {'Windows': 'COM10', 'RaspberryPi': '/dev/ttyACM0', 'Linux': '/dev/ttyUSB0'}[platform.system()] 
ANGLE_OFFSET = np.pi / 2    # = 90°
FORMAT = 'npy'              # 'npy' or 'csv'
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
