import utime, serial
from machine import I2C, Pin
from mpu9250 import MPU9250
import math

i2c = I2C(scl=Pin(22), sda=Pin(21))
sensor = MPU9250(i2c)

ser = serial.Serial('/dev/ttyS0', 9600,timeout=1)


print(sensor.acceleration)

def accelerationToPitchRoll(acceleration):

    ax, ay, az = acceleration

    pitch = math.atan2(ax, math.sqrt(ay**2 + az**2)) * 180 / math.pi
    roll = math.atan2(ay, math.sqrt(ax**2 + az**2)) * 180 / math.pi

    return pitch, roll

while True:
    pitch, roll = accelerationToPitchRoll(sensor.acceleration)
    print("pitch: ", end="")
    print(pitch)
    print("roll: ", end="")
    print(roll)
    ser.wrtie(roll.encode('utf-8'))