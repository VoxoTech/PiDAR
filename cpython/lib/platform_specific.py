import sys
import os


def get_platform():
    platform = sys.platform
    # print("sys.platform:", platform)

    if "win32" in platform:
        platform = 'Windows'
    elif 'darwin' in platform:
        platform = 'Mac'
    else:
        machine = os.uname().machine
        # print("os.uname():",  os.uname())

        if 'linux' in platform:
            if 'aarch64' in machine:
                platform =  'RaspberryPi'
            else:
                platform = 'Linux'
        elif 'Pico W' in machine:
            # 'Raspberry Pi Pico W with rp2040'
            return 'Pico W'
        elif 'Pico' in machine:
            return 'Pico'
        elif 'MIMXRT1011' in machine:
            # 'Metro MIMXRT1011 with IMXRT1011DAE5A'
            return 'Metro M7'
    return platform


platform = get_platform()

if platform in ['Windows', 'Mac', 'Linux', 'RaspberryPi']:
    import numpy as np

elif platform in ['Pico', 'Pico W', 'Metro M7']:
    import board                    # type: ignore
    from ulab import numpy as np    # type: ignore
    

def boardpin(pin):
    board = sys.modules['board']
    return getattr(board, pin)

def init_serial(port='/dev/ttyUSB0'):
    ''' USB: "/dev/ttyUSB0"  GPIO: "/dev/ttyS0" '''
    import serial
    return serial.Serial(port=port, baudrate=230400, timeout=1.0, bytesize=8, parity='N', stopbits=1)

def init_serial_MCU(pin='GP1'):
    from busio import UART          # type: ignore
    return UART(None, boardpin(pin), baudrate=230400, bits=8, parity=None, stop=1)
    

def init_pwm_Pi(pwm_channel=0):
    '''pwm_channel 0: GP18, pwm_channel 1: GP19'''
    from rpi_hardware_pwm import HardwarePWM
    return HardwarePWM(pwm_channel=pwm_channel, hz=20000, chip=0)

def init_pwm_MCU(pin="GP2"):
    from pwmio import PWMOut        # type: ignore
    return PWMOut(boardpin(pin), frequency=20000)


if __name__ == "__main__":
    platform = get_platform()

    print("platform:", platform)
