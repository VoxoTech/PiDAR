"""
https://codefather.tech/blog/shell-command-python/
https://codefather.tech/blog/exit-bash-script/

IMX477 (Raspberry Pi HQ Camera)
https://www.arducam.com/sony/imx477/
"""
import os
import subprocess
import cv2
from time import time
import exifread
import numpy as np


def take_photo(path=None, 
               dims=(4056,3040), 
               exposure_time=None, 
               gain=None,
               denoise="cdn_hq", # cdn_hq, cdn_off, cdn_fast
               awbgains=None,
               awb="auto",
               sharpness=0.5,
               saturation=0.7,
               save_raw=False, 
               blocking=True):
    

    # default path from timestamp
    if path is None:
        path = os.path.join("images", str(time()) + ".jpg")

    params = {
        "--immediate":  "",
        "--nopreview":  "",
        "--output":     path,
        "--denoise":    denoise,
        "--width":      dims[0],
        "--height":     dims[1],
        "--sharpness":  sharpness,
        "--saturation": saturation,
        "--quality":    100,
        "--rotation":   0}  # or 180°


    if awbgains:
        awbgains = ','.join(map(str, awbgains))  # convert to string and add to params
        params["--awbgains"]  = awbgains  
    elif awb.lower() in ["auto", "incadescent", "tungsten", "fluorescent", "indoor", "daylight", "cloudy"]:
        params["--awb"]  = awb
    else:
        raise ValueError("Invalid white balance mode or red/blue gains.")

    if exposure_time:
        params["--shutter"] = int(exposure_time * 1000000)  # to microseconds

    if gain:
        params["--gain"] = gain  # "--analoggain", "--digitalgain" ?

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
    # if not blocking and retval.returncode != 0:
    #     error_message = retval.stderr.read().decode()
    #     raise Exception(f"Command failed with return code {retval.returncode}: {error_message}")

    return path


class ExifReader:
    def __init__(self, path):
        self.path = path
        self.tags = self.read_exif()
        self.exposure_time = self.tags["EXIF ExposureTime"].values[0].num / self.tags["EXIF ExposureTime"].values[0].den
        self.gain = self.tags["EXIF ISOSpeedRatings"].values[0] / 100

    def read_exif(self):
        with open(self.path, 'rb') as f:  # Open image file in binary mode
            tags = exifread.process_file(f)  # Return Exif tags
        return tags

    def print_tags(self):
        for tag in self.tags.keys():
            if tag not in ('JPEGThumbnail', 'TIFFThumbnail', 'Filename', 'EXIF MakerNote'):
                print(f"Key:{tag}, value {self.tags[tag]}")


def r_b_gains(img1, img2):  # source, target
    (b1, _, r1) = cv2.split(img1.astype("float32"))
    (b2, _, r2) = cv2.split(img2.astype("float32"))

    R = r2.mean() / r1.mean()
    B = b2.mean() / b1.mean()
    return R, B


def estimate_camera_parameters(threshold=0.1, max_iterations=10, set_gain=None, bins=32):
    preview_dims = (320, 240)
    awbgains = [1, 1]
    awb_channels = [2,0]  # R, B
    preview_denoise = "cdn_off"

    tmp_dir="images/tmp"
    os.makedirs(tmp_dir, exist_ok=True)

    # take photo in automatic mode
    path_auto = take_photo(path=os.path.join(tmp_dir,"awb.jpg"),
                           dims=preview_dims, 
                           denoise=preview_denoise, 
                           awb="auto", 
                           blocking=True)
    
    exif_auto = ExifReader(path_auto)

    img_auto = cv2.imread(path_auto)
    # hists1 = __calculate_RGB_histogram__(img_auto, bins=bins, channels=awb_channels)
    
    cv2.imshow("auto", img_auto)
    cv2.waitKey(10)

    for i in range(max_iterations):
        # take photo with manual exposure, gain and awbgains
        path_awbgains = take_photo(path=os.path.join(tmp_dir, f"img{i}.jpg"),
                                   dims=preview_dims,
                                   denoise=preview_denoise,
                                   exposure_time=exif_auto.exposure_time,
                                   gain=exif_auto.gain,
                                   awbgains=awbgains,
                                   blocking=True)
        
        img_awbgains = cv2.imread(path_awbgains)
        cv2.imshow("awbgains:", img_awbgains)
        cv2.waitKey(10)

        R, B = awbgains(img_awbgains, img_auto)
        print(R, B)

        awbgains[0] = awbgains[0] * R
        awbgains[1] = awbgains[1] * B
        
        # hists2 = __calculate_RGB_histogram__(img_awbgains, bins=bins, channels=awb_channels)

        # # calculate Bhattacharyya distance for each channel
        # diff_r, diff_b = __compare_RGB_histograms__(hists1, hists2)

        # # Adjust awbgains values based on difference
        # if diff_r > threshold:
        #     awbgains[0] *= 1.0 - 0.1 * np.sign(np.mean(hists1[0]) - np.mean(hists2[0]))
        # if diff_b > threshold:
        #     awbgains[1] *= 1.0 - 0.1 * np.sign(np.mean(hists1[1]) - np.mean(hists2[1]))


    # compute new exposure time based on custom gain
    if set_gain:
        gain = set_gain
        exposure_time = exif_auto.exposure_time * exif_auto.gain / set_gain
    else:
        gain = exif_auto.gain
        exposure_time = exif_auto.exposure_time

    return exposure_time, gain, awbgains


if __name__ == "__main__":
    # path_auto = take_photo(dims=(640, 480), denoise="cdn_off", awb="auto", blocking=True)
    # exif_auto = ExifReader(path_auto)
    # print("ExposureTime:", exif_auto.exposure_time, "| Gain:", exif_auto.gain)


    exposure_time, gain, awbgains = estimate_camera_parameters(set_gain=1, threshold=0.05, max_iterations=10, bins=32)
    print("[RESULT] AE:", exposure_time, "| Gain:", gain, "| AWB:", awbgains)

    # take HighRes image using fixed values
    path = take_photo(exposure_time=exposure_time, gain=gain, denoise="cdn_hq", awbgains=awbgains)
