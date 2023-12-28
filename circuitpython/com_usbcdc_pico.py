import usb_cdc
import time

# Enable the data USB serial device
usb_cdc.enable(console=True, data=True)

# A function to read data from the host
def read_data():

    # Check if there is any data waiting to be read
    if usb_cdc.data.in_waiting > 0:

        # Read the data as bytes
        raw = usb_cdc.data.read(usb_cdc.data.in_waiting)

        # Decode the bytes as UTF-8 string
        text = raw.decode("utf-8")

        # Return the text
        return text
    
    # If there is no data, return None
    else:
        return None

# A function to send data to the host
def send_data(text):

    # Encode the text as UTF-8 bytes
    raw = text.encode("utf-8")

    # Write the bytes to the data USB serial device
    usb_cdc.data.write(raw)

# A loop to send and receive data
while True:

    # Read the data from the host
    data = read_data()

    # If there is any data, print it and send it back
    if data is not None:
        print("Received:", data)
        send_data("Echo: " + data)
        
    # Otherwise, send a hello message every second
    else:
        send_data("Hello from CircuitPython!\n")
        time.sleep(1)
