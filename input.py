import gpiozero
import time

pin = gpiozero.DigitalInputDevice(pin=4, pull_up=False)
try:
    while True:
        print(pin.value)
        time.sleep(1)
finally:
    pin.close()