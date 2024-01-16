import sys

def get_platform():
    return 'PC' if sys.platform in ['win32', 'linux'] else 'MCU'

platform = get_platform()

if platform == 'PC':
    import serial
    import numpy as np

elif platform == 'MCU':
    # import board                    # type: ignore
    from ulab import numpy as np    # type: ignore
    from pwmio import PWMOut        # type: ignore
    from busio import UART          # type: ignore


def boardpin(pin):
    board = sys.modules['board']
    return getattr(board, pin)
    

def init_serial(platform=None, port='COM10', pin='GP1'):
    baudrate = 230400
    bits = 8
    stopbits = 1

    if platform == 'PC':
        port = 'COM10'  # {'Windows': 'COM10', 'RaspberryPi': '/dev/ttyACM0', 'Linux': '/dev/ttyUSB0'}
        return serial.Serial(port=port, baudrate=baudrate, timeout=1.0, bytesize=bits, parity='N', stopbits=stopbits)
    
    elif platform == 'MCU':
        return UART(None, boardpin(pin), baudrate=baudrate, bits=bits, parity=None, stop=stopbits)

def init_pwm(pin="GP2"):
    return PWMOut(boardpin(pin), frequency=20000)
