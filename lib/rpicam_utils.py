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


def take_photo(path=None, 
               dims=(4056,3040), 
               exposure_time=None, 
               gain=None,
               denoise="cdn_hq", # cdn_hq, cdn_off, cdn_fast || "off", "cdn", "luma", "average", "auto" ?
               awb="auto",       # string for modes or tuple for manual gains (red, blue)
               sharpness=0.5,
               saturation=0.7,
               save_raw=False, 
               blocking=False):
    

    # default path from timestamp
    if path is None:
        path = os.path.join("images", str(time()) + ".jpg")

    params = {
        "--immediate":  "",
        "--nopreview":  "",
        "--output":     path,
        "--denoise":    denoise,
        "--awb":        awb,
        "--width":      dims[0],
        "--height":     dims[1],
        "--sharpness":  sharpness,
        "--saturation": saturation,
        "--quality":    100,
        "--rotation":   0}  # or 180°


    if awb.lower() in ["auto", "incadescent", "tungsten", "fluorescent", "indoor", "daylight", "cloudy"]:
        params["--awb"]  = awb
    elif isinstance(awb, tuple):  # manual mode (red, blue gains)
        params["--awbgains"]  = ','.join(map(str, awb))  # convert to string and add to params
    else:
        raise ValueError("Invalid white balance mode or manual gains.")

    if exposure_time:
        params["--shutter"] = exposure_time * 1000000  # to microseconds

    if gain:
        params["--analoggain"] = gain  # == "--gain"
        params["--digitalgain"] = 1

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

    # check returncode if the command was successful
    if not blocking and retval.returncode != 0:
        raise Exception(f"Command failed with return code {retval.returncode}: {retval.stderr.decode()}")

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


def estimate_camera_parameters(threshold=0.01, max_iterations=10):

    def __calculate_histogram__(img):
        hist = cv2.calcHist([img], [0, 1, 2], None, [256, 256, 256], [0, 256, 0, 256, 0, 256])
        cv2.normalize(hist, hist)
        return hist

    preview_dims = (640, 480)
    awbgains = (1, 1)
    preview_denoise = "cdn_off"

    tmp_dir="images/tmp"
    os.makedirs(tmp_dir, exist_ok=True)

    # take photo in automatic mode
    path_auto = take_photo(path=os.path.join(tmp_dir,"awb.jpg"),
                           dims=preview_dims, 
                           denoise=preview_denoise, 
                           awb="auto")
    
    hist_auto = __calculate_histogram__(cv2.imread(path_auto))
    exif_auto = ExifReader(path_auto)

    for i in range(max_iterations):
        # take photo with manual exposure, gain and awbgains
        path_awbgains = take_photo(path=os.path.join(tmp_dir, str(i)+".jpg"),
                                   dims=preview_dims,
                                   denoise=preview_denoise,
                                   exposure_time=exif_auto.exposure_time,
                                   gain=exif_auto.gain,
                                   awbgains=awbgains)
        
        hist_awbgains = __calculate_histogram__(cv2.imread(path_awbgains))

        # calculate Bhattacharyya distance
        difference = cv2.compareHist(hist_auto, hist_awbgains, cv2.HISTCMP_BHATTACHARYYA)
        print("difference:", difference)

        if difference < threshold:
            break
        
        # adjust awbgains values based on difference
        awbgains = (awbgains[0] * difference, awbgains[1] * difference)

    return exif_auto.exposure_time, exif_auto.gain, awbgains


if __name__ == "__main__":
    # # get preview image in automatic mode
    # path_auto = take_photo(dims=(640, 480), denoise="cdn_off", awb="auto")
    # exif_auto = ExifReader(path_auto)
    # print("[Image 1] ExposureTime:", exif_auto.exposure_time, "| Gain:", exif_auto.gain)

    exposure_time, gain, awbgains = estimate_camera_parameters(threshold=0.01, max_iterations=10)
    print("auto Exposure:", exposure_time, "| Gain:", gain, "| AWB Gains:", awbgains)

    time.sleep(1)

    # take HighRes image using fixed values
    path = take_photo(exposure_time=exposure_time, gain=gain, awb=awbgains, denoise="cdn_hq")
    exif = ExifReader(path)
    print("fixed Exposure:", exif.exposure_time, "| Gain:", exif.gain)
