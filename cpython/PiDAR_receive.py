import serial
import time
import json
import os
import threading

def read_data(ser):
    ''' 
    Read a line, decode the bytes as UTF-8 string, 
    strip out newline and carriage return characters
    '''
    while True:
        data = ser.readline().decode().strip()
        if data:
            # Parse JSON and save it to a file
            json_data = json.loads(data)
            save_json(json_data, dir=data_dir)

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

# Start a new thread that will continuously read data
# TODO: might need to add synchronization mechanisms like locks or queues
threading.Thread(target=read_data, args=(ser,), daemon=True).start()

while True:
    # The main loop can do other things here without being blocked by the data reading
    time.sleep(1)
