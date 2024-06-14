from __future__ import division
import time
import math
import busio
import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo  # Using servo class

# Set up I2C bus and PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pwm = PCA9685(i2c)
pwm.frequency = 50  # Standard frequency for servos

# Create servo instances
servos = {}
for channel in range(16):
    servos[channel] = servo.Servo(pwm.channels[channel])

# Function to convert angles to PWM values
def AnglesToPWM(x, high_angle, low_angle, high_pwm, low_pwm):
    ratio = (x - low_angle) / (high_angle - low_angle)
    pwm_value = int(ratio * (high_pwm - low_pwm) + low_pwm)
    return max(min(pwm_value, high_pwm), low_pwm)

# Servo control functions for joints, thighs, and calves
def Joint_1(angle):
    servos[10].angle = AnglesToPWM(angle, 135, 45, 500, 260)

def Joint_2(angle):
    servos[5].angle = AnglesToPWM(angle, 135, 45, 260, 500)

def Joint_3(angle):
    servos[2].angle = AnglesToPWM(angle, 135, 45, 500, 260)

def Joint_4(angle):
    servos[13].angle = AnglesToPWM(angle, 135, 45, 260, 500)

def Thigh_1(angle):
    servos[11].angle = AnglesToPWM(angle, 180, 90, 620, 380)

def Thigh_2(angle):
    servos[4].angle = AnglesToPWM(angle, 180, 90, 140, 380)

def Thigh_3(angle):
    servos[1].angle = AnglesToPWM(angle, 180, 90, 620, 380)

def Thigh_4(angle):
    servos[14].angle = AnglesToPWM(angle, 180, 90, 140, 380)

def Calf_1(angle):
    servos[12].angle = AnglesToPWM(angle, 180, 35, 620, 230)

def Calf_2(angle):
    servos[3].angle = AnglesToPWM(angle, 180, 35, 140, 530)

def Calf_3(angle):
    servos[0].angle = AnglesToPWM(angle, 180, 35, 620, 230)

def Calf_4(angle):
    servos[15].angle = AnglesToPWM(angle, 180, 35, 140, 530)

# Control functions for joints, thighs, and calves
def Joints(angle):
    Joint_1(angle)
    Joint_2(angle)
    Joint_3(angle)
    Joint_4(angle)

def Thighs(angle):
    Thigh_1(angle)
    Thigh_2(angle)
    Thigh_3(angle)
    Thigh_4(angle)

def Calfs(angle):
    Calf_1(angle)
    Calf_2(angle)
    Calf_3(angle)
    Calf_4(angle)

# High-level robot movements and behaviors
def LegsUp():
    Joints(90)
    Thighs(180)
    Calfs(180)

def SetUp():
    Joints(90)
    Thighs(90)
    Calfs(180)

def Sit():
    Calfs(30)
    Thighs(170)

def StandUp():
    LegsUp()
    Calfs(30)
    Thighs(170)

def Forward():
    Calf_4(45)
    Calf_1(45)
    Thigh_1(160)
    Thigh_3(160)
    Joint_1(90)
    Joint_3(90)
    Joint_2(120)
    Joint_4(60)
    Thigh_1(135)
    Thigh_3(135)
    Thigh_2(160)
    Thigh_4(160)
    Joint_2(90)
    Joint_4(90)
    Joint_1(120)
    Joint_3(60)
    Thigh_2(135)
    Thigh_4(135)

def ShiftTo(position):
    # Adjust position
    if position == 1:
        Thigh_1(150)
        Calf_1(35)
        Thigh_2(120)
        Joint_2(120)
        Joint_4(50)
    elif position == 2:
        Thigh_2(150)
        Calf_2(35)
        Thigh_1(120)
        Joint_1(120)
        Joint_3(50)
    elif position == 3:
        Thigh_3(150)
        Calf_3(35)
        Thigh_4(120)
        Joint_4(120)
        Joint_2(50)
    elif position == 4:
        Thigh_4(150)
        Calf_4(35)
        Thigh_3(120)
        Joint_3(120)
        Joint_1(50)
    else:
        Joints(90)

def CCW():
    Calf_4(45)
    Calf_1(45)
    Thigh_1(160)
    Thigh_3(160)
    Joint_1(90)
    Joint_3(90)
    Joint_2(135)
    Joint_4(135)
    Thigh_1(135)
    Thigh_3(135)
    Thigh_2(160)
    Thigh_4(160)
    Joint_2(90)
    Joint_4(90)
    Joint_1(35)
    Joint_3(35)
    Thigh_2(135)
    Thigh_4(135)

def CW():
    Calf_4(45)
    Calf_1(45)
    Thigh_1(160)
    Thigh_3(160)
    Joint_1(90)
    Joint_3(90)
    Joint_2(35)
    Joint_4(35)
    Thigh_1(135)
    Thigh_3(135)
    Thigh_2(160)
    Thigh_4(160)
    Joint_2(90)
    Joint_4(90)
    Joint_1(135)
    Joint_3(135)
    Thigh_2(135)
    Thigh_4(135)

def Right():
    Calf_4(45)
    Calf_1(45)
    Thigh_1(160)
    Thigh_3(160)
    Joint_1(90)
    Joint_3(90)
    Joint_2(50)
    Joint_4(120)
    Thigh_1(135)
    Thigh_3(135)
    Thigh_2(160)
    Thigh_4(160)
    Joint_2(90)
    Joint_4(90)
    Joint_1(120)
    Joint_3(50)
    Thigh_2(135)
    Thigh_4(135)

def Left():
    Calf_4(45)
    Calf_1(45)
    Thigh_1(160)
    Thigh_3(160)
    Joint_1(90)
    Joint_3(90)
    Joint_2(120)
    Joint_4(50)
    Thigh_1(135)
    Thigh_3(135)
    Thigh_2(160)
    Thigh_4(160)
    Joint_2(90)
    Joint_4(90)
    Joint_1(50)
    Joint_3(120)
    Thigh_2(135)
    Thigh_4(135)

# Safety check - Uncomment when ready to run control code
# SetUp()
