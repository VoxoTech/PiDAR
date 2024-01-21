'''
host receiver
'''

import serial
import time

ser = serial.Serial('COM13')

size = 64  # byte

start_time = time.time()
bytes_received = 0
START_SEQUENCE = b'\x01\x02\x03\x04'

while True:
    # Read bytes one at a time until we find the start sequence
    buffer = b''
    while buffer[-len(START_SEQUENCE):] != START_SEQUENCE:
        buffer += ser.read(1)

    # Once we've found the start sequence, read the rest of the packet
    data = ser.read(size - len(START_SEQUENCE))
    bytes_received += len(data)
    elapsed_time = time.time() - start_time
    if elapsed_time > 1:  # Calculate transfer rate every second
        transfer_rate = bytes_received / elapsed_time
        print(f'Transfer rate: {transfer_rate / 1024} KB/s')
        start_time = time.time()
        bytes_received = 0
