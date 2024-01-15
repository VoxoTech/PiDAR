'''
CircuitPython
'''

import board                        # type: ignore
from usb_cdc import data as ser     # type: ignore
import struct
from time import time
from json import dumps

from utils.a4988_driver import A4988
from utils.LD06_driver import LD06


def encode_numpy(array, metadata=None):
    # convert metadata to bytes
    metadata_bytes = dumps(metadata).encode('utf-8')

    # convert array to bytes
    array_bytes = array.tobytes()

    start_sequence = b'\x00\x00\x00\x00START'
    end_sequence = b'END\x00\x00\x00\x00'
    
    data = (start_sequence +
            struct.pack('!II', *array.shape) +
            struct.pack('!I', len(array_bytes)) + array_bytes +
            struct.pack('!I', len(metadata_bytes)) + metadata_bytes +
            end_sequence)

    return data


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

z_rotate = 0.5  # degrees

sending = False
printed_waiting = False

while True:
    if not sending and not printed_waiting:
        print("[Waiting]")
        printed_waiting = True
    if ser.in_waiting > 0:
        data = ser.readline().decode().strip()
        if not sending and data == "start":
            print("[Start received]")
            sending = True
            printed_waiting = False
            lidar.set_pwm_dc = pwm_dc
            stepper.move_angle(z_rotate)  # rotate horizontally for next scan

        elif sending and data == "stop":
            print("[Stop received]")
            sending = False
            lidar.set_pwm_dc = pwm_dc_stop

    if sending:

        # read lidar data [x,y,luminance],[...],[...)
        speed, timestamp, points_2d = lidar.read()

        # create metadata
        metadata = {"lidar_time": timestamp, "current_time": time(), "speed": speed}
        print("[Sending] time:", time())  # metadata, points_2d.shape)

        # encode np.array and metadata dictionary to bytes
        binary_data = encode_numpy(points_2d, metadata)

        # send data
        ser.write(binary_data + b"\n")
