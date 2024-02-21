"""
https://codefather.tech/blog/shell-command-python/
https://codefather.tech/blog/exit-bash-script/
"""
import os
import subprocess
from time import time


def take_photo(path=None, save_raw=False, blocking=False):
    if path is None:
        path = os.path.join("images", str(time()) + ".jpg")

    params = {
        "--immediate":  "",
        "--nopreview":  "",
        "--rotation":   0,  # only 180° step
        "--output":     path,
        "--quality":    100,
        "--width":      4056,
        "--height":     3040,
        "--gain":       1,
        "--denoise":    "cdn_hq",  # cdn_off, cdn_fast, cdn_hq
        "--awb":        "auto",
        "--shutter":    100000,  # in microseconds
        "--sharpness":  0.5 ,
        "--saturation": 0.7
    }

    if save_raw:
        params["--raw"] = ""
    
    # build command string
    params["--encoding"] = params['--output'].split('.')[-1]  # get file extension
    cmd_string = "rpicam-still " + " ".join(f"{k} {v}" for k, v in params.items())


    # execute command either blocking or non-blocking
    if blocking:
        retval = subprocess.run(cmd_string, shell=True, capture_output=True, text=True)
    else:
        retval = subprocess.Popen(cmd_string, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


    # # check returncode if the command was successful
    # if retval.returncode != 0:
    #     raise Exception(f"Command failed with return code {retval.returncode}: {retval.stderr.decode()}")

    return path


if __name__ == "__main__":
    take_photo()
