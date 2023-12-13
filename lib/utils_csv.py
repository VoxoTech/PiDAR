import time
import csv

def save_thread(save_dir, x_list, y_list, color):
    filename = f"{save_dir}/{time.time()}.csv"
    delimiter = ";"
    data = list(zip(x_list, y_list, color))

    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f, delimiter=delimiter)
        writer.writerows(data)
