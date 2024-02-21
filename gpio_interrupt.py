import RPi.GPIO as GPIO  # type: ignore
import subprocess


BUTTON_GPIO = 4
CONDA_ENV = "py310"  # None
MAIN_SCRIPT = "/home/pi/Documents/PiDAR/PiDAR.py"

# Define the callback function that runs when the button is pressed
def button_callback(channel):
    # Check if there is an existing process
    global process
    if process is not None:
        return_code = process.poll()
        # If the return code is None, the process is still running
        if return_code is None:
            print("Process is still running, skipping button press")
            return
        else:
            print("Process has finished, return code:", return_code)
            
    # Start a new process with the main script
    call = ["python3", MAIN_SCRIPT] if CONDA_ENV is None else ["conda", "run", "-n", CONDA_ENV, "python", MAIN_SCRIPT]
    process = subprocess.Popen(call)
    print("Started a new process, pid:", process.pid)


# Set up the GPIO mode and the pin direction
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Add the event detection and the callback function
GPIO.add_event_detect(BUTTON_GPIO, GPIO.FALLING, callback=button_callback, bouncetime=200)

# Initialize the process variable
process = None

# Keep the main thread running
try:
    while True:
        pass
except KeyboardInterrupt:
    GPIO.cleanup()
