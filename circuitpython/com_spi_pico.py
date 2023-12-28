import board
import busio
import digitalio

import time

# Initialize SPI
spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)

# Initialize chip select (CS) pin (and start it high)
cs = digitalio.DigitalInOut(board.GP17)
cs.direction = digitalio.Direction.OUTPUT
cs.value = True

# A function to read data from the host
def read_data():

    # Check if there is any data waiting to be read
    if spi.try_lock():

        # Read the data as bytes
        raw = bytearray(1)
        spi.readinto(raw)

        # Release the SPI lock
        spi.unlock()

        # Decode the bytes as UTF-8 string
        text = raw.decode("utf-8")

        # Return the text
        return text
    
    # If there is no data, return None
    else:
        return None

# A function to send data to the host
def send_data(text):

    # Encode the text as UTF-8 bytes
    raw = text.encode("utf-8")

    # Send out SPI message
    cs.value = False
    spi.write(raw)
    cs.value = True

# A loop to send and receive data
while True:

    # Read the data from the host
    data = read_data()

    # If there is any data, print it and send it back
    if data is not None:
        print("Received:", data)
        send_data("Echo: " + data)
        
    # Otherwise, send a hello message every second
    else:
        send_data("Hello from CircuitPython!\n")
        time.sleep(1)
