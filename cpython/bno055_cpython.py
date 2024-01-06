'''
Please ensure that the data received from the Pico is in the correct format 
for conversion from string to NumPy array. The np.fromstring function expects 
a string representation of an array like "1.0,2.0,3.0" (without brackets) 
and uses a comma as the separator.
'''

import serial
import time
import numpy as np
from scipy.integrate import cumtrapz

from lib.open3d_utils import IMU_Visualizer


def read_data(ser):
    data = ser.readline().decode().strip()
    return data

data_port = "COM13"
baudrate = 115200

ser = serial.Serial(port=data_port, baudrate=baudrate, timeout=60)

# Variables to hold accelerometer data and time data
accel_data = []
time_data = []

start_time = time.time()

# Initialize helper classes
visualizer = IMU_Visualizer()

while True:
    if ser.in_waiting > 0:
        # Read linear acceleration from the Pico
        lin_accel_str = read_data(ser)
        lin_accel = np.fromstring(lin_accel_str[1:-1], sep=',')  # Convert string to numpy array
        accel_data.append(lin_accel)
        time_data.append(time.time() - start_time)
        
        # Integrate acceleration to get velocity
        vel_data = cumtrapz(accel_data, time_data, initial=0)
        
        # Integrate velocity to get position
        pos_data = cumtrapz(vel_data, time_data, initial=0)
        
        # Read the quaternion from the Pico
        quaternion_str = read_data(ser)
        quaternion = np.fromstring(quaternion_str[1:-1], sep=',')  # Convert string to numpy array
        
        # Update the visualization with the latest position and orientation
        visualizer.update_visualization(pos_data[-1], quaternion)

