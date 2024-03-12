'''
A4988 datasheet:
http://www.allegromicro.com/~/media/Files/Datasheets/A4988-Datasheet.ashx

microstepping:
https://i.stack.imgur.com/vN7JL.png
'''

import RPi.GPIO as GPIO     # type: ignore
from time import sleep

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
        
        self.current_steps = 0  # internal absolute angle

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
    
    def get_angle_for_steps(self, steps):
        return (steps / (self.microsteps * self.gear_ratio)) * self.step_angle

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
        
        # update current steps considering direction
        self.current_steps += steps if not direction else -steps

    def move_angle(self, angle):
        steps = self.get_steps_for_angle(abs(angle))
        self.move_steps(steps if angle >= 0 else -steps)
        return steps

    def move_to_angle(self, target_angle, mod=True):
        if mod:
            target_angle %= 360  # keep the target angle within the range of 0 to 359
        current_angle = self.get_current_angle()
        angle_difference = target_angle - current_angle

        # If the angle difference is greater than 180, subtract 360 to get a negative angle
        if angle_difference > 180:
            angle_difference -= 360
        elif angle_difference < -180:
            angle_difference += 360

        # Convert the angle difference to steps and move the motor
        steps_difference = self.get_steps_for_angle(angle_difference)
        self.move_steps(steps_difference)

    def get_current_angle(self, mod=True):
        # Convert current steps to angle before returning
        current_angle = self.current_steps / (self.microsteps * self.gear_ratio) * self.step_angle

        if mod:
            current_angle %= 360
        return current_angle
    
    def close(self):
        #GPIO.setmode(GPIO.BCM)
        GPIO.cleanup(self.ms_pins)
        GPIO.cleanup(self.dir_pin)
        GPIO.cleanup(self.step_pin)


if __name__ == "__main__":
    import numpy as np

    DIR_PIN = 26
    STEP_PIN = 19
    MS_PINS = [5, 6, 13]

    TARGET_RES = 1                              # desired resolution in degrees
    STEP_DELAY = 0.0005
    GEAR_RATIO = 1 + 38/14                      # planetary gear reduction ratio: 3.7142857
    STEPPER_RESOLUTION = 200
    STEP_ANGLE = 360 / STEPPER_RESOLUTION       # 1.8
    MICROSTEPS = 16                             # microstepping mode

    SAMPLING_RATE = 4500                        # samples/second
    SCAN_ANGLE = 180                            # 180° CW or -180° CCW

    scan_delay = 1 / (SAMPLING_RATE * TARGET_RES / 360)  # JUST FOR TESTING
    
    # initialize stepper
    stepper = A4988(DIR_PIN, STEP_PIN, MS_PINS, delay=STEP_DELAY, step_angle=STEP_ANGLE, microsteps=MICROSTEPS, gear_ratio=GEAR_RATIO)

    microsteps_per_revolution = STEPPER_RESOLUTION * MICROSTEPS * GEAR_RATIO
    steps = int(microsteps_per_revolution * TARGET_RES / 360)       # 16
    h_res = 360 * steps / microsteps_per_revolution                 # 0.48464451


    # # TEST: MOVE FULL 360°
    while True:
        stepper.move_steps(microsteps_per_revolution)
        sleep(0.1)
        stepper.move_steps(-microsteps_per_revolution)
        sleep(0.1)

    try:
        # 360° SHOOTING PHOTOS
        for i in range(4):
            print(f"Photo {i+1} at {round(stepper.get_current_angle(), 2)}°")
            sleep(0.5) # delay for photo
            stepper.move_to_angle(90 * (i+1))
        
        stepper.move_to_angle(0)
        stepper.move_steps(1)  # compensate negative value caused by rounding
        sleep(1)

        # 180° SCAN
        print(f"starting angle: {round(stepper.get_current_angle(), 2)}°")
        start_angle = 0 if SCAN_ANGLE > 0 else abs(SCAN_ANGLE)
        for z_angle in np.linspace(start_angle, SCAN_ANGLE, int(abs(SCAN_ANGLE)/h_res), endpoint=False):
            sleep(scan_delay) # duration of one lidar duration
            stepper.move_steps(steps if SCAN_ANGLE > 0 else -steps)

        print(f"reached {round(stepper.get_current_angle(), 2)}° (current steps: {stepper.current_steps}), returning ..")
        stepper.move_to_angle(0)
        
    finally:
        stepper.close()
