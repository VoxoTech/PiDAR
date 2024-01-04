import serial
import time
import json
import os

from utils.plot_2D import plot_2D 


def read_data(ser):
    data = ser.readline().decode().strip()
    return data

def send_data(raw=None, text=None):
    if raw is None and text is not None:
        raw = text.encode("utf-8")
    ser.write(raw)

def save_json(json_data, dir="."):
    path = os.path.join(dir, f"{time.time()}.json")
    with open(path, "w") as f:
        json.dump(json_data, f)


data_port = "COM13"
data_dir = "cpython/data"
baudrate = 115200

ser = serial.Serial(port=data_port, baudrate=baudrate, timeout=60)

# Send the 'start' command as bytes
send_data(raw=b"start\n")

# Initialize the matplotlib visualization
visualisation = plot_2D()

while True:
    if ser.in_waiting > 0:
        data = read_data(ser)
        if data:
            # Parse JSON and save it to a file
            json_data = json.loads(data)
            save_json(json_data, dir=data_dir)
            
            # Parse the data into lists
            x_list = json_data["x"]
            y_list = json_data["y"]
            luminance_list = json_data["luminance"]

            # Update the visualization
            visualisation.update_data(x_list, y_list, luminance_list)
