'''
A4988 datasheet:
http://www.allegromicro.com/~/media/Files/Datasheets/A4988-Datasheet.ashx

microstepping:
https://i.stack.imgur.com/vN7JL.png
'''

import board
import digitalio
import time


class A4988:
    def __init__(self, dir_pin, step_pin, ms_pins, delay=0.0001, step_angle=1.8, microsteps=16, gear_ratio=1.0):
        
        self.dir_pin = digitalio.DigitalInOut(dir_pin)
        self.dir_pin.direction = digitalio.Direction.OUTPUT

        self.step_pin = digitalio.DigitalInOut(step_pin)
        self.step_pin.direction = digitalio.Direction.OUTPUT

        self.ms_pins = [digitalio.DigitalInOut(pin) for pin in ms_pins]
        for pin in self.ms_pins:
            pin.direction = digitalio.Direction.OUTPUT

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
                pin.value = state

    def set_direction(self, direction):
        self.dir_pin.value = direction

    def step(self):
        self.step_pin.value = True
        time.sleep(self.delay)
        self.step_pin.value = False
        time.sleep(self.delay)

    def move_steps(self, steps):
        direction = steps > 0
        steps = abs(int(steps))
        #print("steps:", steps, "direction:", direction)
        
        self.set_direction(direction)
        for _ in range(steps):
            self.step()

    def move_angle(self, angle):
        steps = int((angle / self.step_angle) * self.microsteps * self.gear_ratio)
        self.move_steps(steps)


if __name__ == "__main__":
    dir_pin = board.GP15
    step_pin = board.GP14
    ms_pins = [board.GP11, board.GP12, board.GP13]

    driver = A4988(dir_pin, step_pin, ms_pins, delay=0.0001, step_angle=1.8, microsteps=16, gear_ratio=3.7142857)

    while True:
        driver.move_angle(360)
        time.sleep(0.2)
        
        driver.move_angle(-360)
        time.sleep(0.2)
