
def sample_color(img, uv, normalize_color=False):
    longitude, latitude = uv

    if isinstance(longitude, float):
        h, w, c = img.shape
        longitude = int(longitude * w)
        latitude = int(latitude * h)

    # print(longitude, latitude)
    color = img[latitude, longitude]

    if normalize_color:  # convert uint8 to float
        color = color / 255

    return tuple(color)
