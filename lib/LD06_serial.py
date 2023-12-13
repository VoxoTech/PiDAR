'''
LD06 specification of serial dataframe:
https://storage.googleapis.com/mauser-public-images/prod_description_document/2021/315/8fcea7f5d479f4f4b71316d80b77ff45_096-6212_a.pdf
'''

import platform
import serial

def LD06_serial():
    port = {'Windows': 'COM10', 
            'Linux': '/dev/ttyUSB0'}[platform.system()]  # '/dev/tty.usbserial-0001'
    return serial.Serial(port=port, baudrate=230400, timeout=5.0, bytesize=8, parity='N', stopbits=1)
