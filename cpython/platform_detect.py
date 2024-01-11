'''
https://docs.python.org/3/library/platform.html
https://docs.circuitpython.org/en/latest/docs/library/platform.html

'''

import sys
sys_platform = sys.platform

def detect_model() -> str:
    with open('/proc/device-tree/model') as f:
        model = f.read()
    return model


if sys_platform not in ['win32', 'linux', 'darwin']:  # ['RP2040', 'NXP IMXRT10XX']:
    import os
    info = os.uname()
    # sysname='rp2040',     machine='Raspberry Pi Pico W with rp2040'
    # sysname='mimxrt10xx', machine='Metro MIMXRT1011 with IMXRT1011DAE5A'
    
    if info.sysname == 'rp2040':
        if 'Pico W' in info.machine:
            print("Raspberry Pico W")
        else:
            print("Raspberry Pico")

    elif info.sysname == 'mimxrt10xx':
        print("Adafruit Metro M7")
    else:
        print("unspecified CircuitPython")


elif sys_platform == 'win32':
    import platform
    print(platform.system(), platform.release(), platform.machine())  # Windows 10 AMD64


elif sys_platform == 'linux':
    import os
    info = os.uname()
    # sysname='Linux', version='#14-Ubuntu SMP...', machine='x86_64'
    if "Ubuntu" in info.version:
        print(info.sysname, "Ubuntu", info.machine)  # Linux Ubuntu x86_64
    
    else:
        model = detect_model()
        print("detect_model():", detect_model())

        if 'raspberry' in model.lower():
            print("Raspberry Pi")


elif sys_platform == 'darwin':
    pass

