# pip install pyserial

import serial

ser = serial.Serial('COM5', 115200)

while True:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').rstrip()
        print(line)
