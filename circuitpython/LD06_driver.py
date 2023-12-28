'''
LD06 LiDAR data processing on Raspberry Pi Pico

serial protocol: 1+46 bytes per package (Start gets excluded):
[1B Start (fixed: 0x54)]
1B Data Length = points per package (last 5 digits of 0x2c) fixed: 12
2B Speed in degrees per second (LSB, MSB)
2B Start Angle in 0.01 degree (LSB, MSB)
12x 3B Data (2B Distance (LSB, MSB), 1B Luminance)
2B End Angle in 0.01 degree (LSB, MSB)
2B TimeStamp in ms (recount if reaching to MAX 30000) (LSB, MSB)
1B CRC


output of the circuitpython version:
2c 60 11 e5 66 a5 02 e5 a5 02 e8 c4 02 e5 d4 02 e6 e3 02 e5 02 03 e6 12 03 e6 31 03 e8 41 03 e7 6f 03 e5 8f 03 e4 bd 03 e4 15 6b 92 0e 4b

output of the windows version incl start and end byte at the end -> [0,-5]
0e 0e ce 3a 31 00 ed 39 00 ed 3d 00 ec 36 00 ec 3c 00 ec 35 00 ed 32 00 ed 30 00 ed 39 00 ee 34 00 ec 31 00 ec 39 00 eb 46 3e 28 33 d8 54 2c

https://storage.googleapis.com/mauser-public-images/prod_description_document/2021/315/8fcea7f5d479f4f4b71316d80b77ff45_096-6212_a.pdf
'''

import pwmio
import busio
import board
import math
import binascii


def LD06_pwm(pwm_pin, duty_cycle=0.4):
    LD06_pwm = pwmio.PWMOut(pwm_pin, frequency=20000)
    LD06_pwm.duty_cycle = int(duty_cycle * 65534)
    return LD06_pwm


def LD06_serial(Rx_pin):
    return busio.UART(None, Rx_pin, baudrate=230400, bits=8, parity=None, stop=1)


def LD06_decode(data_package):
    # Extract the fields from the data_package according to the protocol
    speed = int.from_bytes(data_package[0:2], 'big') / 100
    FSA = int.from_bytes(data_package[2:4], 'big') / 100
    data = int.from_bytes(data_package[4:40], 'big') / 100
    LSA = int.from_bytes(data_package[40:42], 'big') / 100
    timestamp = int.from_bytes(data_package[42:44], 'big') % 3000
    CS = int.from_bytes(data_package[44:45], 'big')

    angleStep = (LSA - FSA) / 11 if LSA - FSA > 0 else (LSA + 360 - FSA) / 11

    angle_list = list()
    distance_list = list()
    luminance_list = list()

    for i in range(0, 12):
        angle = (angleStep * i + FSA) % 360
        angle_list.append(angle)

        distance = int.from_bytes(data_package[4 + i*3:6 + i*3], 'big') / 100
        distance_list.append(distance)

        luminance = int.from_bytes(data_package[6 + i*3:7 + i*3], 'big')
        luminance_list.append(luminance)
        
    return speed, FSA, LSA, timestamp, CS, angle_list, distance_list, luminance_list


def polar2cartesian(angles, distances, offset):
    angles = [a + offset for a in angles]
    x_list = [d * -math.cos(a) for a, d in zip(angles, distances)]
    y_list = [d * math.sin(a) for a, d in zip(angles, distances)]
    return x_list, y_list


# constants:
uart_pin = board.GP1
pwm_pin = board.GP2
pwm_dc = 0.1

angle_offset = math.pi / 2  # 90°
data_len = 12  # 12 datapoints x 3 bytes
package_len = 46  # bytes total package size

output_len = 100 # 2D coordinates

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
        
        # Error handling
        if data_package[0] != 44:  # 0x2c
            # print("[WARNING] no Data Length Byte (0x2c) found")
            continue
        # Check the length of data_package
        elif len(data_package) < package_len:
            # print("[WARNING] Incomplete data package")
            continue

        
#         # DEBUG
#         hex_string = binascii.hexlify(data_package).decode()
#         spaced_string = ' '.join([hex_string[i:i+2] for i in range(0, len(hex_string), 2)])
#         print(spaced_string)

        
        # drop first element (0x2c) of the bytearray and decode the remaining
        speed, FSA, LSA, timestamp, CS, angle_batch, distance_batch, luminance_batch = LD06_decode(data_package[1:])
        
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
