'''
A4988 datasheet:
http://www.allegromicro.com/~/media/Files/Datasheets/A4988-Datasheet.ashx

microstepping:
https://i.stack.imgur.com/vN7JL.png
'''

import RPi.GPIO as GPIO     # type: ignore
import time

class A4988:
    def __init__(self, dir_pin, step_pin, ms_pins, delay=0.0001, step_angle=1.8, microsteps=16, gear_ratio=1.0, verbose=False):
        # Disable warnings
        GPIO.setwarnings(verbose) 

        GPIO.setmode(GPIO.BCM)
        
        self.dir_pin = dir_pin
        GPIO.setup(self.dir_pin, GPIO.OUT)

        self.step_pin = step_pin
        GPIO.setup(self.step_pin, GPIO.OUT)

        self.ms_pins = ms_pins
        for pin in self.ms_pins:
            GPIO.setup(pin, GPIO.OUT)

        self.step_angle = step_angle
        self.microsteps = microsteps
        self.gear_ratio = gear_ratio
        self.delay = delay

        self.step_modes = {1: [False, False, False],
                           2: [True, False, False],
                           4: [False, True, False],
                           8: [True, True, False],
                           16:[True, True, True]}

        if self.microsteps in self.step_modes:
            for pin, state in zip(self.ms_pins, self.step_modes[self.microsteps]):
                GPIO.output(pin, state)

    def set_direction(self, direction):
        GPIO.output(self.dir_pin, direction)

    def get_steps_for_angle(self, angle):
        return int((angle / self.step_angle) * self.microsteps * self.gear_ratio)

    def step(self):
        GPIO.output(self.step_pin, True)
        time.sleep(0.0001)
        GPIO.output(self.step_pin, False)
        time.sleep(self.delay)

    def move_steps(self, steps):
        direction = steps < 0   # < clockwise positive
        steps = abs(int(steps))
        
        self.set_direction(direction)
        for _ in range(steps):
            self.step()

    def move_angle(self, angle):
        steps = self.get_steps_for_angle(angle)
        self.move_steps(steps)
        return steps
    
    def close(self):
        GPIO.setmode(GPIO.BCM)  # necessary to avoid "RuntimeError: Please set pin numbering mode" ?
        GPIO.cleanup(self.ms_pins)
        GPIO.cleanup(self.dir_pin)
        GPIO.cleanup(self.step_pin)


if __name__ == "__main__":
    import numpy as np

    dir_pin = 26
    step_pin = 19
    ms_pins = [5, 6, 13]

    ms = 16                                     # microstepping mode
    ms_table = {16: 11885, 8: 5942, 4: 2971}    # table of microsteps per revolution
    ms360 = ms_table[ms]                        # microsteps per revolution

    target_res = 0.5                            # desired resolution in degrees
    delay = 0.0005
    gear_ratio = 3.7142857
    stepper_resolution = 200
    step_angle = 360 / stepper_resolution       # 1.8

    # initialize stepper
    stepper = A4988(dir_pin, step_pin, ms_pins, delay=delay, step_angle=step_angle, microsteps=ms, gear_ratio=gear_ratio)

    steps = int(ms360 * target_res / 360)       # 16
    h_res = 360 * ms / ms360                    # 0.48464451
    scan_delay = 1 / (4500 * target_res / 360)  # 0.16

    # stepper.move_steps(11885)                   # 360°

    try:
        # 0-180° scanning
        for z_angle in np.arange(0, 180, h_res):
            stepper.move_steps(steps)
            time.sleep(scan_delay)              # duration of one lidar duration
        stepper.move_steps(steps)               # n+1 step

        time.sleep(1)

        # 180-360° only shooting photos
        for i in range(4):
            stepper.move_angle(45)
            time.sleep(1)                       # delay for photo

    finally:
        stepper.close()
