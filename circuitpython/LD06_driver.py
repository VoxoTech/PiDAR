'''
LD06 LiDAR data processing on Raspberry Pi Pico

serial protocol (48 bytes):
1B Start, 2B DataLen, 2B Speed, 2B SAngle, 12x 3B DataByte, 2B EAngle, 2B TimeStamp, 1B CRC
https://storage.googleapis.com/mauser-public-images/prod_description_document/2021/315/8fcea7f5d479f4f4b71316d80b77ff45_096-6212_a.pdf
'''

import pwmio
import busio
import board
import math


def LD06_pwm(pwm_pin, duty_cycle=0.4):
    LD06_pwm = pwmio.PWMOut(pwm_pin, frequency=30000)
    LD06_pwm.duty_cycle = int(duty_cycle * 65534)
    return LD06_pwm


def LD06_serial(Rx_pin):
    return busio.UART(None, Rx_pin, baudrate=230400, bits=8, parity=None, stop=1)


def LD06_decode(data_package):
    # Extract the fields from the data_package according to the protocol
    speed = int.from_bytes(data_package[2:4], 'big') / 100
    FSA = int.from_bytes(data_package[4:6], 'big') / 100
    LSA = int.from_bytes(data_package[-8:-6], 'big') / 100
    timestamp = int.from_bytes(data_package[-4:-2], 'big')
    CS = int.from_bytes(data_package[-2:], 'big')

    angleStep = (LSA - FSA) / (data_len-1) if LSA - FSA > 0 else (LSA + 360 - FSA) / (data_len-1)
    
    angle_batch = list()
    distance_batch = list()
    luminance_batch = list()
    
    # Process the data points
    for i in range(8, 8 + data_len * 3, 3):
        angle = ((angleStep * (i // 3) + FSA) % 360) * math.pi / 180.0
        angle_batch.append(angle)

        distance = int.from_bytes(data_package[i:i+2], 'big') / 100
        distance_batch.append(distance)

        luminance = data_package[i+2]
        luminance_batch.append(luminance)
        
    return speed, FSA, LSA, timestamp, CS, angle_batch, distance_batch, luminance_batch


def polar2cartesian(angles, distances, offset):
    angles = [a + offset for a in angles]
    x_list = [d * -math.cos(a) for a, d in zip(angles, distances)]
    y_list = [d * math.sin(a) for a, d in zip(angles, distances)]
    return x_list, y_list


# constants:
uart_pin = board.GP1
pwm_pin = board.GP2
pwm_dc = 0.4

angle_offset = math.pi / 2  # 90°
data_len = 12  # 12 datapoints x 3 bytes
package_len = 48  # bytes total package size
output_len = 24 # 2D coordinates

# initialize LD06 PWM speed control
LD06_pwm = LD06_pwm(pwm_pin, duty_cycle=pwm_dc)

# initialize lists
angle_list = list()
distance_list = list()
luminance_list = list()
x_list = list()
y_list = list()
        
# receive serial data
with LD06_serial(uart_pin) as serial_connection:
    i = 0
    while True:            
        # Detect the start of the package
        while True:
            start_byte = serial_connection.read(1)
            if start_byte is not None and int.from_bytes(start_byte, 'big') == 0x54:
                break

        # Read the entire package
        data_package = serial_connection.read(package_len)

        # Check the length of data_package
        if len(data_package) < package_len:
            print("[WARNING] Incomplete data package")
            continue
        
        # decode bytearray 
        speed, FSA, LSA, timestamp, CS, angle_batch, distance_batch, luminance_batch = LD06_decode(data_package)
        
        # append batch of 12 values to global list
        angle_list += angle_batch
        distance_list += distance_batch
        luminance_list += luminance_batch
        
        # calculate cartesian coordinates from angles and distances
        x_batch, y_batch = polar2cartesian(angle_batch, distance_batch, angle_offset)
        x_list += x_batch
        y_list += y_batch
        
        # return larger package of data and clear lists
        if i % output_len == output_len - 1:
            print(f'Speed: {speed}, FSA {FSA}, LSA: {LSA}, Timestamp: {timestamp}, CS: {CS}, values: {len(angle_list)}')
            # print(f'Lists | X: {x_list}, Y: {y_list}, Luminance: {luminance_list}')
            
            # prepare for next iteration
            angle_list.clear()
            distance_list.clear()
            luminance_list.clear()
            x_list.clear()
            y_list.clear()
            i = 0

        i += 1
