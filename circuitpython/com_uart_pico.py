import time
import board
import busio

# Function to simulate sensor data collection
def simulate_sensor():
   return int(time.monotonic()) % 4500

uart = busio.UART(board.GP4, board.GP5, baudrate=115200)

while True:
    data = simulate_sensor()
    print(data)
    uart.write(str(data).encode())
    time.sleep(1)
