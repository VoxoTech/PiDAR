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


def estimate_camera_parameters(threshold=0.1, max_iterations=10, set_gain=None, bins=32):

    def __calculate_channel_histogram__(img, channel, bins=256):
        hist = cv2.calcHist([img], [channel], None, [bins], [0, 256])
        cv2.normalize(hist, hist)
        return hist

    def __calculate_RGB_histogram__(img, bins=256):
        hist_r = __calculate_channel_histogram__(img, 0, bins)
        hist_g = __calculate_channel_histogram__(img, 1, bins)
        hist_b = __calculate_channel_histogram__(img, 2, bins)
        return hist_r, hist_g, hist_b

    preview_dims = (320, 240)
    awbgains = [1, 1]
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
    hist1_r, hist1_g, hist1_b = __calculate_RGB_histogram__(img_auto, bins=bins)
    
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

        hist2_r, hist2_g, hist2_b = __calculate_RGB_histogram__(img_awbgains, bins=bins)

        # calculate Bhattacharyya distance for each channel
        diff_r = cv2.compareHist(hist1_r, hist2_r, cv2.HISTCMP_BHATTACHARYYA)
        diff_g = cv2.compareHist(hist1_g, hist2_g, cv2.HISTCMP_BHATTACHARYYA)
        diff_b = cv2.compareHist(hist1_b, hist2_b, cv2.HISTCMP_BHATTACHARYYA)

        diff = (diff_r + diff_g + diff_b) / 3

        print("iteration:", i+1, "| diff:", round(diff, 2), "(", round(diff_r,2), round(diff_g,2), round(diff_b,2), ")")

        if diff < threshold:
            break
        
        # TODO: adjust awbgains values based on difference
        awbgains = (awbgains[0] + 0.2, awbgains[1] + 0.5)

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
