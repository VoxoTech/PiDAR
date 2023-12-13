"""
https://codefather.tech/blog/shell-command-python/
https://codefather.tech/blog/exit-bash-script/
"""

import os

cmd_string = "libcamera-still -o long_exposure.jpg -q 100 --shutter 1000000 --awb indoor --gain 1 \
                --immediate --sharpness 0 --denoise cdn_off --raw"

retval = os.popen(cmd_string).read()
print(retval)
