# PiDAR
sophisticated 360° 3D Panorama Scanner: LDRobot LD06 LiDAR @ Raspberry Pi Pico | HQ Camera + M12 Fisheye Lens + Hugin @ Raspberry Pi 4 | Nema17 @ A4988

## core features:
- optimized LD06 serial/UART driver (Python / CircuitPython)
- export 2D data (including luminance) as csv
- cartesian 2D visualisation (matplotlib)
- 3D conversion based on Z-rotation 
- interactive 3D visualisation (Open3D)

influenced by:
- [LIDAR_LD06_python_loder](https://github.com/henjin0/LIDAR_LD06_python_loder) and [Lidar_LD06_for_Arduino](https://github.com/henjin0/Lidar_LD06_for_Arduino) by Inoue Minoru ("[henjin0](https://github.com/henjin0)")
- [ShaunPrice's](https://github.com/ShaunPrice/360-camera) StereoPi-supporting fork of [BrianBock's](https://github.com/BrianBock/360-camera) 360-camera script


### Serial Protocol
baudrate 230400, data bits 8, no parity, 1 stopbit  
sampling frequency 4500, scan frequency 5-13 Hz, distance 2cm - 12 meter, ambient light 30 kLux

total package size: 48 Byte, big endian.
- starting character：Length 1 Byte, fixed value 0x54, means the beginning of data packet;
- Data Length: Length 1 Byte, the first three digits reserved, the last five digits represent the number of measured points in a packet, currently fixed value 12;
- speed：Length 2 Byte, in degrees per second;
- Start angle: Length: 2 Byte; unit: 0.01 degree;
- Data: Length 36 Byte; containing 12 data points with 3 Byte each: 2 Byte distance (unit: 1 mm), 1 Byte luminance. For white objects within 6m, the typical luminance is around 200.
- End Angle: Length: 2 Byte; unit: 0.01 degree；
- Timestamp: Length 2 Bytes in ms, recount if reaching to MAX 30000；
- CRC check: Length 1 Byte

The Angle value of each data point is obtained by linear interpolation of the starting angle and the ending angle.  
The calculation method of the angle is as following:
> step = (end_angle – start_angle)/(len – 1)  
> angle = start_angle + step*i  

len is the length of the packet, and the i value range is [0, len].


## About LD06(LDS06)
- [Sales page](https://www.inno-maker.com/product/lidar-ld06/)
- [mechanical Datasheet](https://www.inno-maker.com/wp-content/uploads/2020/11/LDROBOT_LD06_Datasheet.pdf)
- [Protocol Description](https://storage.googleapis.com/mauser-public-images/prod_description_document/2021/315/8fcea7f5d479f4f4b71316d80b77ff45_096-6212_a.pdf)
- another potentially interesting implementation: [pyLIDAR](https://github.com/Paradoxdruid/pyLIDAR)