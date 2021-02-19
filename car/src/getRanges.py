import time
import board
import busio
import digitalio
from adafruit_vl53l0x import VL53L0X
from stepperMotor import rotateMotor

#tof sensor setup
i2c = busio.I2C(board.SCL, board.SDA)

vl53_1 = VL53L0X(i2c, address=0x33)
vl53_2 = VL53L0X(i2c, address=0x34)
vl53_3 = VL53L0X(i2c, address=0x35)
vl53_4 = VL53L0X(i2c, address=0x36)

#stepper motor pins
IN1 = digitalio.DigitalInOut(board.D14)
IN2 = digitalio.DigitalInOut(board.D15)
IN3 = digitalio.DigitalInOut(board.D18)
IN4 = digitalio.DigitalInOut(board.D23)

IN1.direction = digitalio.Direction.OUTPUT
IN2.direction = digitalio.Direction.OUTPUT
IN3.direction = digitalio.Direction.OUTPUT
IN4.direction = digitalio.Direction.OUTPUT

#initialize arrays for each sensor
sensor_data1 = []
sensor_data2 = []
sensor_data3 = []
sensor_data4 = []
 
currentStep = 0
currentDegrees = 0

rotationCount = 171 #15 degrees

for i in range(6):
    currentStep, currentDegrees = rotateMotor(currentStep, rotationCount, -1, currentDegrees, IN1, IN2, IN3, IN4)
    sensor_data1.append(vl53_1.range)
    sensor_data2.append(vl53_2.range)
    sensor_data3.append(vl53_3.range)
    sensor_data4.append(vl53_4.range)
    time.sleep(1)


currentStep, currentDegrees = rotateMotor(currentStep, rotationCount*5, 1, currentDegrees, IN1, IN2, IN3, IN4) #reset motor position

# write results in order
f = open("ranges.txt", "w")

for i in range(6):
    f.write(f"{sensor_data1[i]}, ")
f.write("\n")
for i in range(6):
    f.write(f"{sensor_data2[i]}, ")
f.write("\n")
for i in range(6):
    f.write(f"{sensor_data3[i]}, ")
f.write("\n")
for i in range(6):
    f.write(f"{sensor_data4[i]}, ")
f.write("\n")


f.close()
