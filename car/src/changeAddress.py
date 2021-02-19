import time
import board
import busio
from digitalio import DigitalInOut
from adafruit_vl53l0x import VL53L0X

i2c = busio.I2C(board.SCL, board.SDA)

xshut = [
    DigitalInOut(board.D17),
    DigitalInOut(board.D27),
    DigitalInOut(board.D22),
    DigitalInOut(board.D10),
]

for power_pin in xshut:
    power_pin.switch_to_output(value=False)

xshut[0].value = True
vl53 = VL53L0X(i2c)
vl53.set_address(0x33)

time.sleep(.1)

xshut[1].value = True
vl53_2 = VL53L0X(i2c)
vl53_2.set_address(0x34)

time.sleep(.1)

xshut[2].value = True
vl53_2 = VL53L0X(i2c)
vl53_2.set_address(0x35)

time.sleep(.1)

xshut[3].value = True
vl53_2 = VL53L0X(i2c)
vl53_2.set_address(0x36)

