import numpy as np
import RPi.GPIO as GPIO  # type: ignore
from time import sleep

from lib.platform_utils import allow_serial
from lib.matplotlib_2D import plot_2D
from lib.lidar_driver import LD06  #, STL27L
from lib.a4988_driver import A4988
from lib.file_utils import save_data, make_dir


# allow access to serial port on Raspberry Pi
allow_serial()

# GPIO SETUP
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# LiDAR DRIVER
PORT = '/dev/ttyS0'                         # {'Windows': 'COM10', 'RaspberryPi': '/dev/ttyS0', 'Linux': '/dev/ttyUSB0'}  # dmesg | grep "tty"
PWM_DC = 0.2                                # duty cycle: 0.23 = 10 Hz, 0.1 = 4 Hz
OFFSET = np.pi / 2                          # = 90°
FORMAT = 'npy'                              # 'npy' or 'csv' or None
DTYPE = np.float32                          # np.float64 or np.float32
DATA_DIR = "data"
VIS = plot_2D()                             # plot_2D() or None
OUT_LEN = 40                                # visualize after every nth batch

# A4988 DRIVER
DIR_PIN = 26
STEP_PIN = 19
MS_PINS = [5, 6, 13]
MICROSTEPS = 16                             # microstepping mode
MS_TABLE = {16: 11885, 8: 5942, 4: 2971}    # table of microsteps per revolution
MS360 = MS_TABLE[MICROSTEPS]                # microsteps per revolution
TARGET_RES = 0.5                            # desired resolution in degrees
STEP_DELAY = 0.0005                         # seconds between steps
GEAR_RATIO = 3.7142857                      # planetary gear reduction ratio
STEP_ANGLE = 1.8                            # degrees per full step (360 / 200)

steps = int(MS360 * TARGET_RES / 360)       # 16
h_res = 360 * steps / MS360                 # 0.48464451
scan_delay = 1 / (4500 * TARGET_RES / 360)  # 0.16

# Power Relay
RELAY_PIN = 24
GPIO.setup(RELAY_PIN, GPIO.OUT)
GPIO.output(RELAY_PIN, 1)                   # Relay Power on

# ensure output directory
make_dir(DATA_DIR)

# initialize devices
lidar   = LD06(port=PORT, pwm_dc = PWM_DC, visualization=VIS, offset=OFFSET,format=FORMAT, dtype=DTYPE, data_dir=DATA_DIR, out_len=OUT_LEN)
stepper = A4988(DIR_PIN, STEP_PIN, MS_PINS, delay=STEP_DELAY, step_angle=STEP_ANGLE, microsteps=MICROSTEPS, gear_ratio=GEAR_RATIO)


# MAIN
try:
    # 0-180° SCAN
    for z_angle in np.arange(0, 180, h_res):

        if lidar.serial_connection.is_open:
        
            if lidar.out_i == lidar.out_len:
                # print("speed:", round(lidar.speed, 2))
                
                # SAVE DATA
                if lidar.format is not None:
                    save_data(lidar.data_dir, lidar.points_2d, lidar.format)

                # VISUALIZE
                if lidar.visualization is not None:
                    lidar.visualization.update_coordinates(lidar.points_2d)
                lidar.out_i = 0

            # READ LIDAR DATA FROM SERIAL
            lidar.read()
            lidar.out_i += 1

            stepper.move_steps(steps)
            sleep(scan_delay)                   # duration of one lidar duration
        stepper.move_steps(steps)               # n+1 step

        sleep(1)
        
        # 180-360° JUST SHOOTING PHOTOS
        for i in range(4):
            stepper.move_angle(45)
            sleep(1)                            # delay for photo

finally:
    print("SCANNING STOPPED")
    GPIO.output(24, 0)                          # Relay Power off
    GPIO.cleanup()
    lidar.close()
    stepper.close()
