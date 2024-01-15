import serial
import time
import json
import os
import struct
import numpy as np
import keyboard

from lib.matplotlib_utils import plot_2D 


def read_data(ser):
    start_sequence = b'\x00\x00\x00\x00START'
    end_sequence = b'END\x00\x00\x00\x00'

    # Read until the start sequence is found
    while True:
        data = ser.read(len(start_sequence))
        if data == start_sequence:
            break

    # Read the shape of the numpy array
    shape_bytes = ser.read(8)
    shape = struct.unpack('!II', shape_bytes)

    # Read the size of the numpy array data
    size_bytes = ser.read(4)
    array_size = struct.unpack('!I', size_bytes)[0]

    # Read the numpy array data
    array_bytes = ser.read(array_size)
    array_data = np.frombuffer(array_bytes, dtype=np.float32).reshape(shape)

    # Read the size of the JSON payload
    size_bytes = ser.read(4)
    json_size = struct.unpack('!I', size_bytes)[0]

    # Read the JSON payload
    json_bytes = ser.read(json_size)
    metadata = json.loads(json_bytes.decode('utf-8'))

    # Read until the end sequence is found
    while True:
        data = ser.read(len(end_sequence))
        if data == end_sequence:
            break

    return metadata, array_data


def send_data(ser, raw=None, text=None):
    if raw is None and text is not None:
        raw = text.encode("utf-8")
    ser.write(raw)
    print("[Sent]", raw)


def save_data(path, metadata, np_array):
    now = str(time.time())
    
    npy_path = os.path.join(path, now + ".npy")
    np.save(npy_path, np_array)

    json_path = os.path.join(path, now + ".json")
    with open(json_path, "w") as f:
        json.dump(metadata, f)


if __name__ == "__main__":

    data_port = "COM13"
    data_dir = "cpython/data"
    baudrate = 115200  # 230400

    ser = serial.Serial(port=data_port, baudrate=baudrate, timeout=60)

    # Send the 'start' command as bytes
    send_data(ser, raw=b"start\n")

    # Initialize the matplotlib visualization
    visualisation = plot_2D()

    while ser.is_open and keyboard.is_pressed('q') is False:

        if ser.in_waiting > 0:

            metadata, points_2d = read_data(ser)

            if metadata and points_2d is not None:
                
                print("[Received]", metadata, points_2d.shape, points_2d[0])

                save_data(data_dir, metadata, points_2d)
                
                # Update the visualization
                visualisation.update_coordinates(points_2d)

    ser.close()
    print("Serial connection closed.")
