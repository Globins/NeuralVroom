import time

import board
import busio

import adafruit_vl53l0x


i2c = busio.I2C(board.SCL, board.SDA)
vl53 = adafruit_vl53l0x.VL53L0X(i2c)

vl53.set_address(0x30)

