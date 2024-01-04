import pwmio
import busio
import board
import math
import binascii
import time


class LD06:
    def __init__(self, uart_pin, pwm_pin, pwm_dc=0.4):
        self.uart_pin = uart_pin
        self.pwm_pin = pwm_pin
        self.pwm_dc = pwm_dc
        self.pwm = self.init_pwm()
        
        self.package_len = 46
        self.output_len = 50  # x12 2D coordinates
        self.len_byte = 44  # 0x2c
        
        self.angle_offset = math.pi / 2  # 90°


    def init_pwm(self):
        pwm = pwmio.PWMOut(self.pwm_pin, frequency=20000)
        pwm.duty_cycle = int(self.pwm_dc * 65534)
        return pwm
    
    
#     def set_pwm_dc(self, dc):
#         self.pwm_dc = dc
#         self.LD06_pwm.duty_cycle = int(dc * 65534)


    def LD06_serial(self, Rx_pin):
        return busio.UART(None, Rx_pin, baudrate=230400, bits=8, parity=None, stop=1)


    def LD06_decode(self, data_package):
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


    def polar2cartesian(self, angles, distances, offset):
        angles = [a + offset for a in angles]
        x_list = [d * -math.cos(a) for a, d in zip(angles, distances)]
        y_list = [d * math.sin(a) for a, d in zip(angles, distances)]
        return x_list, y_list


    def read_data(self):
        angle_list = list()
        distance_list = list()
        luminance_list = list()
        x_list = list()
        y_list = list()
    
        with self.LD06_serial(self.uart_pin) as serial_connection:
            i = 0
            while True:
                # Detect the start of the package
                while True:
                    start_byte = serial_connection.read(1)
                    if start_byte is not None and int.from_bytes(start_byte, 'big') == 0x54:
                        break

                # Read the entire package
                data_package = serial_connection.read(self.package_len)
                
                # Error handling
                if data_package[0] != self.len_byte:  # 0x2c
                    # print("[WARNING] no Data Length Byte (0x2c) found")
                    continue
                # Check the length of data_package
                elif len(data_package) < self.package_len:
                    # print("[WARNING] Incomplete data package")
                    continue
                
                # drop first element (0x2c) of the bytearray and decode the remaining
                speed, FSA, LSA, timestamp, CS, angle_batch, distance_batch, luminance_batch = self.LD06_decode(data_package[1:])
                
                # append batch of 12 values to global list
                angle_list += angle_batch
                distance_list += distance_batch
                luminance_list += luminance_batch
                
                # calculate cartesian coordinates from angles and distances
                x_batch, y_batch = self.polar2cartesian(angle_batch, distance_batch, self.angle_offset)
                x_list += x_batch
                y_list += y_batch
                
                # return larger package of data and clear lists
                if i % self.output_len == self.output_len - 1:
                    
                    #print(f'Speed: {speed}, FSA {FSA}, LSA: {LSA}, Timestamp: {timestamp}, CS: {CS}, values: {len(angle_list)}')
                    #print(f'Lists | X: {x_list}, Y: {y_list}, Luminance: {luminance_list}')
                    return x_list, y_list, luminance_list
                
#                     # prepare for next iteration
#                     angle_list.clear()
#                     distance_list.clear()
#                     luminance_list.clear()
#                     x_list.clear()
#                     y_list.clear()
#                     i = 0
                i += 1



if __name__ == "__main__":
    
    lidar = LD06(board.GP1, board.GP2, pwm_dc=0.3)

    while True:
        x_list, y_list, luminance_list = lidar.read_data()
        print(len(x_list))