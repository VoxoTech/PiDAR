import RPi.GPIO as GPIO  
from time import sleep

GPIO.setmode(GPIO.BCM)
#GPIO.setup(25, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(24, GPIO.OUT)
  
try:  
    while True:
        #if GPIO.input(25):
        GPIO.output(24, 1)
        sleep(1.0)
        #else:  
        GPIO.output(24, 0)
        sleep(1.1)
  
finally:
    GPIO.cleanup()
