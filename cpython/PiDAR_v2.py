import threading
import keyboard

from lib.LD06_driver import LD06_v2
from lib.matplotlib_utils import plot_2D
from lib.file_utils import save_thread


data_dir = "cpython/data"
output_len = 4500
save_csv = True
csv_delimiter = ";"

# port = {'Windows': 'COM10', 'RaspberryPi': '/dev/ttyACM0', 'Linux': '/dev/ttyUSB0'}[platform.system()]  
lidar = LD06_v2('COM10')

visualisation = plot_2D()

# inits
x_list = []
y_list = []
luminance_list = []
counter = 0

while keyboard.is_pressed('q') is False:
    x_batch, y_batch, luminance_batch = lidar.read()

    x_list += x_batch
    y_list += y_batch
    luminance_list += luminance_batch
    counter += len(x_batch)

    if counter >= output_len:
        visualisation.update_data(x_list, y_list, luminance_list)

        if save_csv:
            t = threading.Thread(target=save_thread, args=(data_dir, x_list, y_list, luminance_list, csv_delimiter))
            t.start()
    
        x_list.clear()
        y_list.clear()
        luminance_list.clear()
        counter = 0

lidar.close()
print("Application closed.")
