import time
import board
import busio
import adafruit_bno055
import usb_cdc
import toml
import math


# Initialize I2C bus and BNO055
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

# File path for the calibration file
CALIBRATION_FILE = "/BNO055_calibration.toml"

loop_pause = 0.1  # seconds


# Function to load calibration data from a file
def load_calibration_data():
    try:
        with open(CALIBRATION_FILE, "r") as f:
            calibration_data = toml.load(f)
        return calibration_data
    except FileNotFoundError:
        return None


# Function to convert quaternion to Euler angles (if needed later)
def quaternion_to_euler(q):
    # Assuming the quaternion is (w, x, y, z)
    w, x, y, z = q
    ysqr = y * y
    
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.degrees(math.atan2(t0, t1))
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.degrees(math.atan2(t3, t4))
    
    return X, Y, Z  # roll (X), pitch (Y), yaw (Z)


# Function to set the desired mode
def set_fusion_mode(mode='ndof'):
    if mode == 'ndof':
        sensor.mode = adafruit_bno055.NDOF_MODE
    else: #  mode == 'imu':
        sensor.mode = adafruit_bno055.IMUPLUS_MODE

set_fusion_mode('ndof')


# Load calibration data if available, else perform calibration
calibration_data = load_calibration_data()

# CALIBRATION PROCESS
if not calibration_data:
    print("Starting calibration process...")
    print("For gyroscope, ensure the sensor is still")
    print("For accelerometer, place the sensor in 6 positions")
    print("For magnetometer, perform 'figure 8' movements or normal varied movements")

    while True:
        # Display current calibration status
        system, gyro, accel, mag = sensor.get_calibration_status()
        print(f"Calibration status: Sys={system}, Gyro={gyro}, Accel={accel}, Mag={mag}")

        if system == 3:
            calibration_data = sensor.get_calibration()
            with open(CALIBRATION_FILE, "w") as f:
                toml.dump(calibration_data, f)
            print("Calibration complete!")
            break
        else:
            time.sleep(1)

# LOAD CALIBRATION DATA FROM FILE
else:
    sensor.set_calibration(calibration_data)



# Main loop
while True:
    # Read linear acceleration and quaternion from the BNO055
    lin_accel = sensor.linear_acceleration
    quaternion = sensor.quaternion

    # Optionally convert quaternion to Euler angles
    # euler_angles = quaternion_to_euler(quaternion)

    # Send data to the host PC
    usb_cdc.data.write(f"LinAccel: {lin_accel}\nQuat: {quaternion}\n".encode())

    # Sleep for the desired measurement frequency (adjust as needed)
    time.sleep(loop_pause)
