import RPi.GPIO as GPIO  # type: ignore
import numpy as np
from time import sleep

from lib.platform_utils import allow_serial
# from lib.matplotlib_2D import plot_2D
from lib.lidar_driver import LD06  #, STL27L
from lib.a4988_driver import A4988
from lib.rpicam_utils import take_photo, estimate_camera_parameters, ExifReader
from lib.pano_utils import hugin_stitch
from lib.file_utils import save_data, make_dir


# allow access to serial port on Raspberry Pi
allow_serial()

DEBUG = False

# GPIO SETUP
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(DEBUG)

# ENABLES
enable_camera = False
enable_stitching = True                     # requires enable_camera
enable_lidar = True

# LiDAR DRIVER
PORT = '/dev/ttyS0'                         # {'Windows': 'COM10', 'RaspberryPi': '/dev/ttyS0', 'Linux': '/dev/ttyUSB0'}  # dmesg | grep "tty"
PWM_DC = 0.23                               # duty cycle: 0.23 = 6.5 Hz, 0.1 = 4 Hz
OFFSET = np.pi / 2                          # = 90°
FORMAT = 'npy'                              # 'npy' or 'csv' or None
DTYPE = np.float64                          # np.float64 or np.float32
DATA_DIR = "data"
VIS = None  # plot_2D()                             # plot_2D() or None
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

# photos
IMGCOUNT = 4
raw = False
imglist = []
template_path = f"panocam/template_{IMGCOUNT}.pto"
PANO_WIDTH = 3600
set_gain = 1
awb_thres=0.01

packages_per_revolution = round(4500 / 12 / 10)  # 38
hsteps = 180 / h_res
max_packages = hsteps * packages_per_revolution


# calibrate camera
if enable_camera:
    # extract exposure time and gain from exif data, iterate through Red/Blue Gains for custom AWB
    exposure_time, gain, awbgains = estimate_camera_parameters(set_gain=set_gain, awb_thres=awb_thres)
    if DEBUG:
        print("[RESULT] AE:", exposure_time, "| Gain:", gain, "| AWB R:", round(awbgains[0],3), "B:", round(awbgains[1],3))

# ensure output directory
make_dir(DATA_DIR)

# initialize stepper
stepper = A4988(DIR_PIN, STEP_PIN, MS_PINS, delay=STEP_DELAY, step_angle=STEP_ANGLE, microsteps=MICROSTEPS, gear_ratio=GEAR_RATIO)

def move_steps_callback():
    stepper.move_steps(steps)


# initialize lidar
if enable_lidar:
    lidar   = LD06(port=PORT, pwm_dc = PWM_DC, visualization=VIS, offset=OFFSET,format=FORMAT, dtype=DTYPE, data_dir=DATA_DIR, out_len=OUT_LEN)


# MAIN
try:
    # 360° SHOOTING PHOTOS
    if enable_camera:
        for i in range(IMGCOUNT):
            stepper.move_angle(360/IMGCOUNT)

            # take HighRes image using fixed values
            imgpath = take_photo(exposure_time=exposure_time, gain=gain, awbgains=awbgains, denoise="cdn_hq", save_raw=raw, blocking=True)
            imglist.append(imgpath)
            # sleep(1)
    
    # STITCHING PROCESS (NON-BLOCKING)
    if enable_stitching and enable_camera:
        project_path = hugin_stitch(imglist, template=template_path, width=PANO_WIDTH)

    # 180° SCAN
    if enable_lidar:
        lidar.read_loop(callback=move_steps_callback, max_packages=max_packages)

        # for z_angle in np.arange(0, 180, h_res):
        #     if lidar.serial_connection.is_open:
        #         print("Z angle:", round(z_angle, 2))
        #         print("Speed:", round(lidar.speed, 2))

        #         # SAVE DATA
        #         if lidar.format is not None:
        #             save_data(lidar.data_dir, lidar.points_2d, lidar.format)

        #         if lidar.out_i == lidar.out_len:
        #             # VISUALIZE
        #             if lidar.visualization is not None:
        #                 lidar.visualization.update_coordinates(lidar.points_2d)
        #             lidar.out_i = 0

        #         # READ LIDAR DATA FROM SERIAL
        #         sleep(scan_delay)
        #         lidar.read()
        #         lidar.out_i += 1
        #         stepper.move_steps(steps)

        stepper.move_steps(steps)  # last step to complete 180°
        stepper.move_angle(180)    # complete revolution

finally:
    print("PiDAR STOPPED")
    if enable_lidar:
        lidar.close()
    stepper.close()

    # Relay Power off
    GPIO.output(RELAY_PIN, 0)                          
    GPIO.cleanup(RELAY_PIN)
