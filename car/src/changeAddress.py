import time
import board
import busio
from digitalio import DigitalInOut
from adafruit_vl53l0x import VL53L0X

i2c = busio.I2C(board.SCL, board.SDA)

xshut = [
    DigitalInOut(board.D15),
    DigitalInOut(board.D18),
]

for power_pin in xshut:
    power_pin.switch_to_output(value=False)

xshut[0].value = True
vl53 = VL53L0X(i2c)
vl53.set_address(0x30)

time.sleep(.1)

xshut[1].value = True
vl53_2 = VL53L0X(i2c)
vl53_2.set_address(0x31)

