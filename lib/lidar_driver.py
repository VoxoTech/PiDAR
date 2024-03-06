'''
LD06 datasheet:
https://storage.googleapis.com/mauser-public-images/prod_description_document/2021/315/8fcea7f5d479f4f4b71316d80b77ff45_096-6212_a.pdf

375 batches/s x 12 samples/batch = 4500 samples/s
'''

import numpy as np
import serial
import os
from time import sleep

try:
    # running from project root
    from lib.platform_utils import get_platform, init_serial, init_serial_MCU, init_pwm_Pi, init_pwm_MCU
    from lib.file_utils import save_data
except:
    # testing from this file
    from platform_utils import get_platform, init_serial, init_serial_MCU, init_pwm_Pi, init_pwm_MCU
    from file_utils import save_data


class LD06:
    def __init__(self, port=None, pwm_channel=0, pwm_dc=0.4, baudrate=230400, offset=0, data_dir="data", out_len=40, format=None, visualization=None, dtype=np.float32):
        self.platform           = get_platform()

        self.z_angle            = 0  # gets updated externally by A4988 driver

        # constants
        self.start_byte         = bytes([0x54])
        self.dlength_byte       = bytes([0x2c])
        self.dlength            = 12  # 12 samples per batch
        self.package_len        = 47  # 45 byte + 0x54 + 0x2c
        self.deg2rad            = np.pi / 180
        self.offset             = offset

        # serial
        self.port               = port
        if self.platform in ['Pico', 'Pico W', 'Metro M7']:
            self.serial_connection  = init_serial_MCU(pin=self.port, baudrate=baudrate)
        else:  # self.platform in ['Windows', 'Linux', 'RaspberryPi']:
            self.serial_connection  = init_serial(port=self.port, baudrate=baudrate)
        
        self.byte_array         = bytearray()
        self.flag_2c            = False
        self.dtype              = dtype

        self.out_len            = out_len
        # preallocate batch:
        self.timestamp          = 0
        self.speed              = 0
        self.angle_batch        = np.zeros(self.dlength)
        self.distance_batch     = np.zeros(self.dlength)
        self.luminance_batch    = np.zeros(self.dlength)
        # preallocate outputs:
        self.out_i              = 0
        self.speeds             = np.empty(self.out_len, dtype=self.dtype)
        self.timestamps         = np.empty(self.out_len, dtype=self.dtype)
        self.points_2d          = np.empty((self.out_len * self.dlength, 3), dtype=self.dtype)  # [[x, y, l],[..
        
        # file format and visualization
        self.data_dir           = data_dir
        self.format             = format
        self.visualization      = visualization

        self.pwm                = None
        self.pwm_dc             = pwm_dc
        self.pwm_frequency      = 30000  # 30 kHz

        # platform-specific
        if self.platform in ['Pico', 'Pico W', 'Metro M7']:
            pwm_pin             = "GP2"
            pwm                 = init_pwm_MCU(pwm_pin, frequency=self.pwm_frequency)
            pwm.duty_cycle      = int(self.pwm_dc * 65534)
        
        elif self.platform == 'RaspberryPi':
            pwm                 = init_pwm_Pi(pwm_channel, frequency=self.pwm_frequency)
            pwm.start(int(self.pwm_dc * 100))
            # pwm.change_duty_cycle(50)            

            # disable OpenGL
            os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'  # opengl_fallback(check=False)
            
    
    def close(self):
        if self.pwm is not None:
            self.pwm.stop()
            print("PWM stopped.")

        if self.visualization is not None:
            self.visualization.close()
            print("Visualization closed.")

        self.serial_connection.close()
        print("Serial connection closed.")
    

    def read_loop(self, callback=None, max_packages=None):
        loop_count = 0
        if self.visualization is not None:
            # matplotlib close event
            def on_close(event):
                self.serial_connection.close()
                print("Closing...")
            self.visualization.fig.canvas.mpl_connect('close_event', on_close)

        # TODO: hack; wait for other processes to calm down
        sleep(2)

        while self.serial_connection.is_open and (max_packages is None or loop_count <= max_packages):
            try:
                if self.out_i == self.out_len:
                    # print("speed:", round(self.speed, 2))
                    
                    if callback is not None:
                        callback()
                
                    # SAVE DATA
                    if self.format is not None:
                        filepath = os.path.join(self.data_dir, f"image_{round(self.z_angle, 2)}.{self.format}")
                        save_data(filepath, self.points_2d)

                    # VISUALIZE
                    if self.visualization is not None:
                        self.visualization.update_coordinates(self.points_2d)

                    self.out_i = 0

                self.read()

            except serial.SerialException:
                print("SerialException")
                break

            self.out_i += 1
            loop_count += 1


    def read(self):
        # iterate through serial stream until start package is found
        while self.serial_connection.is_open:
            data_byte = self.serial_connection.read()

            if data_byte == self.start_byte:
                self.byte_array.extend(data_byte)
                self.flag_2c = True
                continue

            elif data_byte == self.dlength_byte and self.flag_2c:
                self.byte_array.extend(data_byte)

                # Error handling
                if len(self.byte_array) != self.package_len:
                    # print("[WARNING] Incomplete:", self.byte_array)
                    self.byte_array = bytearray()
                    self.flag_2c = False
                    continue
                
                self.decode(self.byte_array)  # updates speed, timestamp, angle_batch, distance_batch, luminance_batch
                x_batch, y_batch = self.polar2cartesian(self.angle_batch, self.distance_batch, self.offset)
                points_batch = np.column_stack((x_batch, y_batch, self.luminance_batch)).astype(self.dtype)

                # write into preallocated output arrays at current index
                self.speeds[self.out_i] = self.speed
                self.timestamps[self.out_i] = self.timestamp
                self.points_2d[self.out_i*self.dlength:(self.out_i+1)*self.dlength] = points_batch

                # reset byte_array
                self.byte_array = bytearray()
                break

            else:
                self.byte_array.extend(data_byte)

            self.flag_2c = False


    def decode(self, byte_array):  
        # dlength = 12  # byte_array[46] & 0x1F
        self.speed = int.from_bytes(byte_array[0:2][::-1], 'big') / 360         # rotational frequency in rps
        FSA = float(int.from_bytes(byte_array[2:4][::-1], 'big')) / 100         # start angle in degrees
        LSA = float(int.from_bytes(byte_array[40:42][::-1], 'big')) / 100       # end angle in degrees
        self.timestamp = int.from_bytes(byte_array[42:44][::-1], 'big')         # timestamp in milliseconds < 30000
        # CS = int.from_bytes(byte_array[44:45][::-1], 'big')                     # TODO: CRC Checksum                                
        
        angleStep = ((LSA - FSA) if LSA - FSA > 0 else (LSA + 360 - FSA)) / (self.dlength-1)

        # 3 bytes per sample x 12 samples
        for counter, i in enumerate(range(0, 3 * self.dlength, 3)): 
            self.angle_batch[counter] = ((angleStep * counter + FSA) % 360) * self.deg2rad
            self.distance_batch[counter] = int.from_bytes(byte_array[4 + i:6 + i][::-1], 'big')  # mm units
            self.luminance_batch[counter] = byte_array[6 + i]


    @staticmethod
    def polar2cartesian(angles, distances, offset):
        angles = list(np.array(angles) + offset)
        x_list = distances * -np.cos(angles)
        y_list = distances * np.sin(angles)
        return x_list, y_list


class STL27L(LD06):
    def __init__(self, port='COM6', pwm_channel=0, pwm_dc=0.4, offset=0, data_dir="data", out_len=200, format=None, visualization=None, dtype=np.float32):
        # call the __init__ method of the parent class with the new baudrate and sampling rate
        super().__init__(port, pwm_channel, pwm_dc, 921600, offset, data_dir, out_len, format, visualization, dtype)
        self.sampling_rate = 21600  # new sampling rate for STL27L


def my_callback():
    print("Callback function called!")


if __name__ == "__main__":

    visualize = True

    if visualize:
        from matplotlib_2D import plot_2D
        visualization = plot_2D()
    else:
        import threading
        visualization = None
    
    # 4500 points per second / 12 points per batch / 10 revolutions per second = 37.5 batches per revolution
    packages_per_revolution = 38
    hsteps = 50
    max_packages = hsteps * packages_per_revolution


    lidar = LD06(port = '/dev/ttyS0',  # 'COM8',
                 pwm_dc = 0.4,
                 offset = np.pi / 2, 
                 format = 'npy',
                 dtype =np.float64,
                 data_dir ="data",
                 visualization = visualization,
                 out_len = packages_per_revolution)  

    try:
        if lidar.serial_connection.is_open:

            if visualize:
                lidar.read_loop(callback=my_callback, max_packages=max_packages)
            else:
                read_thread = threading.Thread(target=lidar.read_loop, kwargs={'callback': my_callback, 'max_packages': max_packages})
                read_thread.start()
                read_thread.join()
    finally:
        lidar.close()
