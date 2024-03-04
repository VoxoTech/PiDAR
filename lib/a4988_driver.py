'''
A4988 datasheet:
http://www.allegromicro.com/~/media/Files/Datasheets/A4988-Datasheet.ashx

microstepping:
https://i.stack.imgur.com/vN7JL.png
'''

import RPi.GPIO as GPIO     # type: ignore
from time import sleep

class A4988:
    def __init__(self, dir_pin, step_pin, ms_pins, delay=0.0001, step_angle=1.8, microsteps=16, gear_ratio=1.0, init_angle=0, verbose=False):
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

        self.current_angle = init_angle

    def set_direction(self, direction):
        GPIO.output(self.dir_pin, direction)

    def get_steps_for_angle(self, angle):
        return int((angle / self.step_angle) * self.microsteps * self.gear_ratio)

    def step(self):
        GPIO.output(self.step_pin, True)
        sleep(0.0001)
        GPIO.output(self.step_pin, False)
        sleep(self.delay)

    def move_steps(self, steps):
        direction = steps < 0   # < clockwise positive
        steps = abs(int(steps))

        self.set_direction(direction)
        for _ in range(steps):
            self.step()
        
        # update current angle
        self.current_angle += self.get_steps_for_angle(steps)

    def move_angle(self, angle):
        steps = self.get_steps_for_angle(angle)
        self.move_steps(steps)
    
        # Update current angle
        self.current_angle += angle

        return steps
    
    def move_to_angle(self, target_angle):
        # Calculate the difference between the current and target angle
        angle_difference = target_angle - self.current_angle

        # Move the motor by the calculated difference
        self.move_angle(angle_difference)

    def get_current_angle(self):
        return self.current_angle
    
    def close(self):
        GPIO.setmode(GPIO.BCM)      # TODO: necessary to avoid "RuntimeError: Please set pin numbering mode" ?
        GPIO.cleanup(self.ms_pins)
        GPIO.cleanup(self.dir_pin)
        GPIO.cleanup(self.step_pin)


if __name__ == "__main__":
    import numpy as np

    DIR_PIN = 26
    STEP_PIN = 19
    MS_PINS = [5, 6, 13]

    MICROSTEPS = 16                             # microstepping mode
    MS_TABLE = {16: 11885, 8: 5942, 4: 2971}    # table of microsteps per revolution
    MS360 = MS_TABLE[MICROSTEPS]                # microsteps per revolution

    TARGET_RES = 0.5                            # desired resolution in degrees
    STEP_DELAY = 0.0005
    GEAR_RATIO = 3.7142857
    STEPPER_RESOLUTION = 200
    STEP_ANGLE = 360 / STEPPER_RESOLUTION       # 1.8

    # initialize stepper
    stepper = A4988(DIR_PIN, STEP_PIN, MS_PINS, delay=STEP_DELAY, step_angle=STEP_ANGLE, microsteps=MICROSTEPS, gear_ratio=GEAR_RATIO)

    steps = int(MS360 * TARGET_RES / 360)       # 16
    h_res = 360 * steps / MS360                 # 0.48464451
    scan_delay = 1 / (4500 * TARGET_RES / 360)  # 0.16

    # # TEST: MOVE FULL 360°
    # stepper.move_steps(MS360)
    
    try:
        # 0-180° SCAN
        for z_angle in np.arange(0, 180, h_res):
            stepper.move_steps(steps)
            sleep(scan_delay)                   # duration of one lidar duration
        stepper.move_steps(steps)               # n+1 step

        sleep(1)

        # 180-360° JUST SHOOTING PHOTOS
        for i in range(4):
            stepper.move_angle(45)
            print(f"Photo {i+1} at {stepper.current_angle}°")
            sleep(1)                            # delay for photo

    finally:
        stepper.close()
