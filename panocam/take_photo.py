"""
https://codefather.tech/blog/shell-command-python/
https://codefather.tech/blog/exit-bash-script/
"""

import os

# cmd_string = "libcamera-still -o long_exposure.jpg -q 100 --shutter 1000000 --awb indoor --gain 1 \
#                 --immediate --sharpness 0 --denoise cdn_off --raw"

cmd_string = "rpicam-still --rotation 180 --immediate --nopreview -e jpg --quality 100 -o test.jpg --width 4056 --height 3040 \
    --gain 1 --denoise cdn_hq --awb auto --shutter 500000 --sharpness 0.5 --saturation 0.9"

retval = os.popen(cmd_string).read()
# print(retval)
