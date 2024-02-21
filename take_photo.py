"""
https://codefather.tech/blog/shell-command-python/
https://codefather.tech/blog/exit-bash-script/
"""

import subprocess


def take_photo(filename, save_raw=False):

    params = {
        "--immediate":  "",
        "--nopreview":  "",
        "--rotation":   0,  # only 180° step
        "--output":     filename,
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
    params["--encoding"] = params['--output'].split('.')[-1]
    cmd_string = "rpicam-still " + " ".join(f"{k} {v}" for k, v in params.items())

    # execute command
    retval = subprocess.run(cmd_string, shell=True, capture_output=True, text=True)
    # print(retval)


if __name__ == "__main__":
    from time import time
    import os

    filename = os.path.join("images", str(time()) + ".jpg")
    take_photo(filename, save_raw=False)
