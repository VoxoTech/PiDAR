import numpy as np
import os
from lib.matplotlib_utils import plot_2D
from lib.LD06_driver import LD06


# CONSTANTS
# dmesg | grep "tty"
PORT = '/dev/ttyS0'  # {'Windows': 'COM10', 'RaspberryPi': '/dev/ttyS0', 'Linux': '/dev/ttyUSB0'}
PWM_DC = 0.23                # duty cycle: 0.23 = 10 Hz, 0.1 = 4 Hz
ANGLE_OFFSET = np.pi / 2    # = 90°
FORMAT = 'npy'              # 'npy' or 'csv' or None
DTYPE = np.float32
DATA_DIR = "data"
VISUALIZATION = plot_2D()   # plot_2D() or None
OUT_LEN = 40                # visualize after every nth batch

# ensure output directory
if not os.path.exists(DATA_DIR):
    os.makedirs(DATA_DIR)

lidar = LD06(port=PORT,
            pwm_dc = PWM_DC,
            visualization=VISUALIZATION,
            offset=ANGLE_OFFSET, 
            format=FORMAT,
            dtype=DTYPE,
            data_dir=DATA_DIR,
            out_len=OUT_LEN)

try:
    if lidar.serial_connection.is_open:
        lidar.read_loop()
finally:
    lidar.close()
