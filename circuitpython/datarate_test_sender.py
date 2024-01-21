import os
from usb_cdc import data as ser     # type: ignore
import time

START_SEQUENCE = b'\x01\x02\x03\x04'
len_start_sequence = len(START_SEQUENCE)

def generate_dummy_data(size):
    return START_SEQUENCE + b'\00' * (size - 4)

size = 64  # byte

start_time = time.time()
bytes_sent = 0

while True:
    dummy_data = generate_dummy_data(size)

    ser.write(dummy_data)
    bytes_sent += len(dummy_data)
    elapsed_time = time.time() - start_time
    
    if elapsed_time > 1:  # Calculate transfer rate every second
        transfer_rate = bytes_sent / elapsed_time
        print(f'Timestamp: {time.time()}, Transfer rate: {transfer_rate / 1024} KB/s')
        start_time = time.time()
        bytes_sent = 0
