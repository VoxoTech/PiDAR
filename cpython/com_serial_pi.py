'''
serial on CPython:
pip install pyserial
'''

import serial
import time


# Open the data USB serial device
ser = serial.Serial(port="COM3", baudrate=115200, timeout=1)

# A function to read data from the device
def read_data():

    # Check if there is any data waiting to be read
    if ser.in_waiting > 0:

        # Read the data as bytes
        raw = ser.read(ser.in_waiting)
        # ser.readline().decode('utf-8').rstrip()

        # Decode the bytes as UTF-8 string
        text = raw.decode("utf-8")

        # Return the text
        return text
    
    # If there is no data, return None
    else:
        return None

# A function to send data to the device
def send_data(text):
    # Encode the text as UTF-8 bytes
    raw = text.encode("utf-8")
    # Write the bytes to the data USB serial device
    ser.write(raw)

# A loop to send and receive data
while True:
    # Read the data from the device
    data = read_data()
    # If there is any data, print it and send it back
    if data is not None:
        print("Received:", data)
        send_data("Echo: " + data)
    # Otherwise, send a hello message every second
    else:
        send_data("Hello from Python!\n")
        time.sleep(1)
