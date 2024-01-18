"""
https://codefather.tech/blog/shell-command-python/
https://codefather.tech/blog/exit-bash-script/
"""

import os


params = {
    # "--raw":        "",
    "--immediate":  "",
    "--nopreview":  "",
    "--rotation":   180,
    "--output":     "image.jpg",
    "--quality":    100,
    "--width":      4056,
    "--height":     3040,
    "--gain":       1,
    "--denoise":    "cdn_hq",  # cdn_off, cdn_fast, cdn_hq
    "--awb":        "auto",
    "--shutter":    500000,  # in microseconds
    "--sharpness":  0.7 ,
    "--saturation": 0.9
}

# build command string and set file type automatically
params["--encoding"] = params['--output'].split('.')[-1]
cmd_string = "rpicam-still " + " ".join(f"{k} {v}" for k, v in params.items())

# execute command
retval = os.popen(cmd_string).read()
