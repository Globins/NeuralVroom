import time
import board
import busio
from digitalio import DigitalInOut
from adafruit_vl53l0x import VL53L0X

i2c = busio.I2C(board.SCL, board.SDA)

vl53_front = VL53L0X(i2c, address=0x30)
vl53_back = VL53L0X(i2c, address=0x31)

f = open("ranges.txt", "w")

for i in range(5):
    f.write(f"{vl53_front.range},{vl53_back.range}\n")

f.close()
