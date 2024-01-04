import os
import time
import board
import json
import usb_cdc

from utils.a4988_driver import A4988
from utils.LD06_driver import LD06

## WIFI
#from utils.wifi_utils import start_session, print_response
#session = start_session(os.getenv('WIFI_SSID'), os.getenv('WIFI_PASSWORD'))
#print_response(session)


# Stepper
dir_pin = board.GP15
step_pin = board.GP14
ms_pins = [board.GP11, board.GP12, board.GP13]
stepper = A4988(dir_pin, step_pin, ms_pins, delay=0.0001, step_angle=1.8, microsteps=16, gear_ratio=3.7142857)

# LiDAR
uart_pin = board.GP1
pwm_pin = board.GP2
pwm_dc = 0.3
pwm_dc_stop = 0.01
lidar = LD06(uart_pin, pwm_pin, pwm_dc=pwm_dc)

loop_duration = 1  # seconds
sendtime = time.monotonic()
sending = False

while True:
    data = usb_cdc.data.readline().decode().strip()
    
    if data == "start":
        sending = True
#         lidar.pwm.duty_cycle = pwm_dc
        
#         stepper.move_angle(5.)

    elif data == "stop":
        sending = False
#         lidar.pwm.duty_cycle = pwm_dc_stop

    if sending and time.monotonic() - sendtime >= loop_duration:
        x_list, y_list, luminance_list = lidar.read_data()
        data = json.dumps({"timestamp": time.time(), "x": x_list, "y": y_list, "luminance": luminance_list})
        usb_cdc.data.write(data.encode() + b"\n")
        sendtime = time.monotonic()
        
    print(f"{time.time()}")