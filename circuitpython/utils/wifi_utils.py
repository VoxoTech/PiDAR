'''
USAGE:

from utils.wifi_utils import start_session, print_response
session = start_session(os.getenv('WIFI_SSID'), os.getenv('WIFI_PASSWORD'))
print_response(session)
'''

import ssl
import wifi                 # type: ignore
import socketpool           # type: ignore
import adafruit_requests    # type: ignore
import microcontroller      # type: ignore


# private function
def get_socketpool(ssid, password):
    wifi.radio.connect(ssid, password)
    pool = socketpool.SocketPool(wifi.radio)
    return pool
    
# main function
def start_session(ssid, password):
    pool = get_socketpool(ssid, password)
    session = adafruit_requests.Session(pool, ssl.create_default_context())
    return session

# adafruit example
def print_response(session, url="https://www.adafruit.com/api/quotes.php"):
    try:
        response = session.get(url)
        response_test = response.text
        print("Text Response: ", response_test)
        response.close()
        return response_test    

    except Exception as e:
        print("Error:\n", str(e))
        print("Resetting microcontroller!")
        microcontroller.reset()
