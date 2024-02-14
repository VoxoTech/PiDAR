import numpy as np
import os

from lib.platform_utils import allow_serial
from lib.matplotlib_2D import plot_2D
from lib.LD06_driver import LD06
from lib.a4988_driver import A4988
from lib.file_utils import save_data


# allow access to serial port on Raspberry Pi
allow_serial()

# CONSTANTS
PORT = '/dev/ttyS0'         # {'Windows': 'COM10', 'RaspberryPi': '/dev/ttyS0', 'Linux': '/dev/ttyUSB0'}  # dmesg | grep "tty"
PWM_DC = 0.1                # duty cycle: 0.23 = 10 Hz, 0.1 = 4 Hz
OFFSET = np.pi / 2          # = 90°
FORMAT = 'npy'              # 'npy' or 'csv' or None
DTYPE = np.float64          # np.float32
DATA_DIR = "data"
VIS = plot_2D()             # plot_2D() or None
OUT_LEN = 40                # visualize after every nth batch

DIR_PIN = 26                # direction pin
STEP_PIN = 19               # step pin
MS_PINS = [5, 6, 13]        # microstepping mode pins

STEP_DELAY = 0.0005         # seconds between steps
STEP_ANGLE = 1.8            # degrees per full step
MICROSTEPS = 16             # microsteps per full step
GEAR_RATIO = 3.7142857      # planetary gear reduction ratio

HORIZONTAL_RESOLUTION = 0.25  # degrees


# ensure output directory
if not os.path.exists(DATA_DIR):
    os.makedirs(DATA_DIR)


lidar   = LD06(port=PORT, pwm_dc = PWM_DC, visualization=VIS, offset=OFFSET,format=FORMAT, dtype=DTYPE, data_dir=DATA_DIR, out_len=OUT_LEN)
stepper = A4988(DIR_PIN, STEP_PIN, MS_PINS, delay=STEP_DELAY, step_angle=STEP_ANGLE, microsteps=MICROSTEPS, gear_ratio=GEAR_RATIO)


# calculate exact amount of steps for the desired angle
steps = stepper.move_angle(HORIZONTAL_RESOLUTION)
print(f"[INFO] {HORIZONTAL_RESOLUTION}° -> {steps} steps.")



try:
    for z_angle in range(0, 360, HORIZONTAL_RESOLUTION):
        if lidar.serial_connection.is_open:
            # lidar.read_loop()  
        
            if lidar.out_i == lidar.out_len:
                # print("speed:", round(lidar.speed, 2))
                
                # SAVE DATA
                if lidar.format is not None:
                    save_data(lidar.data_dir, lidar.points_2d, lidar.format)

                # VISUALIZE
                if lidar.visualization is not None:
                    lidar.visualization.update_coordinates(lidar.points_2d)
                lidar.out_i = 0

            lidar.read()
            lidar.out_i += 1

            stepper.move_steps(steps)

finally:
    lidar.close()
    stepper.close()
