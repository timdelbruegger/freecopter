#!/usr/bin/python

import smbus
import math
import numpy as np
from quadro_common import *

# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def dist(a,b):
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

def x_rotation(v):
    x = v[0]
    y = v[1]
    z = v[2]
    return get_x_rotation(x,y,z)

def y_rotation(v):
    x = v[0]
    y = v[1]
    z = v[2]
    return get_y_rotation(x,y,z)

def xy_rotation(v):
    x = v[0]
    y = v[1]
    z = v[2]
    x_rotation = get_x_rotation(x,y,z)
    y_rotation = get_y_rotation(x,y,z)
    return np.array([x_rotation, y_rotation], float)

bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards
address = 0x68       # This is the address value read via the i2cdetect command

# Now wake the 6050 up as it starts in sleep mode
bus.write_byte_data(address, power_mgmt_1, 0)

def printTestData():
    print ("gyro data")
    print ("---------")

    gyro_xout = read_word_2c(0x43)
    gyro_yout = read_word_2c(0x45)
    gyro_zout = read_word_2c(0x47)

    print("gyro_xout: ", gyro_xout, " scaled: ", (gyro_xout / 131), sep=",")
    print("gyro_yout: ", gyro_yout, " scaled: ", (gyro_yout / 131), sep=",")
    print("gyro_zout: ", gyro_zout, " scaled: ", (gyro_zout / 131), sep=",")

    print()
    print ("accelerometer data")
    print ("------------------")

    accel_xout = read_word_2c(0x3b)
    accel_yout = read_word_2c(0x3d)
    accel_zout = read_word_2c(0x3f)

    accel_xout_scaled = accel_xout / 16384.0
    accel_yout_scaled = accel_yout / 16384.0
    accel_zout_scaled = accel_zout / 16384.0

    print("accel_xout: ", accel_xout, " scaled: ", accel_xout_scaled, sep=",")
    print("accel_yout: ", accel_yout, " scaled: ", accel_yout_scaled, sep=",")
    print("accel_zout: ", accel_zout, " scaled: ", accel_zout_scaled, sep=",")

    print("x rotation: " , get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled), sep=",")
    print("y rotation: " , get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled), sep=",")

def accelerometerLinear():
    accel_xout = read_word_2c(0x3b)
    accel_yout = read_word_2c(0x3d)
    accel_zout = read_word_2c(0x3f)
    return np.array([accel_xout, accel_yout, accel_zout])

def accelerometerLinear_scaled():
    accel = accelerometerLinear()
    accel_scaled = accel / 16384.0
    return accel_scaled

def accelerometerRotation():
    accel_scaled = accelerometerLinear_scaled()
    return xy_rotation(accel_scaled)

def accelerometerRotationReliable():
    accel_scaled = accelerometerLinear_scaled()
    return abs(1-length(accel_scaled))**2

def gyro():
    gyro_xout = read_word_2c(0x43) / 131.0
    gyro_yout = read_word_2c(0x45) / 131.0
    gyro_zout = read_word_2c(0x47) / 131.0
    return np.array([gyro_xout, gyro_yout, gyro_zout])

# unit is m/s^2
gravity_strength = 9.81

# represents one reading from the accelerometer.
# all Values are in m/s^2
class AccelerometerReading:
    def __init__(self):
        self.__linear = accelerometerLinear()/ 16384.0 * gravity_strength

    def linear(self):
        return self.__linear

    # Approximation of the x and y rotation angles,
    # Based on the assumption that the measured acceleration is gravity.
    def rotation_approximation(self):
        return xy_rotation(self.linear())

    def pitch_approximation(self):
        return self.rotation_approximation()[0]

    def roll_approximation(self):
        return self.rotation_approximation()[1]

    # Total acceleration strength in m/s^2
    def strength(self):
        return length(self.linear());

    # acceleration strength relative to gravity
    # something like 3G = 3*gravity = 3*9.81 m/s^2
    def strength_gravity(self):
        return length(self.linear())/gravity_strength

    # Tests the total amount of linear acceleration.
    # If it is much stronger than gravity, the rotation approximation is very weak and should not be used.
    def rotation_approximation_variance(self):
        return abs(1-self.strength_gravity())+1


class GyroReading():
    def __init__(self):
        self.value = gyro()

    def degreesPerSecond(self):
        return self.value

    def radianPerSecond(self):
        return self.degreesPerSecond() / 360 * 2 *math.pi
