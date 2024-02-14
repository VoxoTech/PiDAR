'''
A4988 datasheet:
http://www.allegromicro.com/~/media/Files/Datasheets/A4988-Datasheet.ashx

microstepping:
https://i.stack.imgur.com/vN7JL.png
'''

import RPi.GPIO as GPIO     # type: ignore
import time

class A4988:
    def __init__(self, dir_pin, step_pin, ms_pins, delay=0.0001, step_angle=1.8, microsteps=16, gear_ratio=1.0):
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
                           16: [True, True, True]}

        if self.microsteps in self.step_modes:
            for pin, state in zip(self.ms_pins, self.step_modes[self.microsteps]):
                GPIO.output(pin, state)

    def set_direction(self, direction):
        GPIO.output(self.dir_pin, direction)

    def step(self):
        GPIO.output(self.step_pin, True)
        time.sleep(0.0001)
        GPIO.output(self.step_pin, False)
        time.sleep(self.delay)

    def move_steps(self, steps):
        direction = steps > 0
        steps = abs(int(steps))
        
        self.set_direction(direction)
        for _ in range(steps):
            self.step()

    def move_angle(self, angle):
        steps = int((angle / self.step_angle) * self.microsteps * self.gear_ratio)
        self.move_steps(steps)


if __name__ == "__main__":
    dir_pin = 26
    step_pin = 19
    ms_pins = [5, 6, 13]

    driver = A4988(dir_pin, step_pin, ms_pins, delay=0.0005, step_angle=1.8, microsteps=16, gear_ratio=3.7142857)

    while True:
        driver.move_angle(360)
        time.sleep(0.2)
        
        # driver.move_angle(-360)
        # time.sleep(0.2)
