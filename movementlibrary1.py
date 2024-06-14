from __future__ import division
import time
import math
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

# Set up I2C bus and PCA9685
i2c = busio.I2C(SCL, SDA)
pwm = PCA9685(i2c)
pwm.frequency = 60

# Set servo pulse
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000  # 1,000,000 us per second
    pulse_length //= pwm.frequency
    pulse_length //= 4096  # 12-bit resolution
    pulse_in_counts = int(pulse * 1000 / pulse_length)
    pwm.channels[channel].duty_cycle = pulse_in_counts

# Convert angles to PWM values
def AnglesToPWM(x, high_pwm, low_pwm, high_angle, low_angle):
    ratio = (x - low_angle) / (high_angle - low_angle)
    pwm_value = int(ratio * (high_pwm - low_pwm) + low_pwm)
    return max(min(pwm_value, high_pwm), low_pwm)

# Joint control functions
def Joint_1(angle):
    pwm.channels[10].duty_cycle = AnglesToPWM(angle, 500, 260, 135, 45)

def Joint_2(angle):
    pwm.channels[5].duty_cycle = AnglesToPWM(angle, 260, 500, 135, 45)

def Joint_3(angle):
    pwm.channels[2].duty_cycle = AnglesToPWM(angle, 500, 260, 135, 45)

def Joint_4(angle):
    pwm.channels[13].duty_cycle = AnglesToPWM(angle, 260, 500, 135, 45)

# Thigh control functions
def Thigh_1(angle):
    pwm.channels[11].duty_cycle = AnglesToPWM(angle, 620, 380, 180, 90)

def Thigh_2(angle):
    pwm.channels[4].duty_cycle = AnglesToPWM(angle, 140, 380, 180, 90)

def Thigh_3(angle):
    pwm.channels[1].duty_cycle = AnglesToPWM(angle, 620, 380, 180, 90)

def Thigh_4(angle):
    pwm.channels[14].duty_cycle = AnglesToPWM(angle, 140, 380, 180, 90)

# Calf control functions
def Calf_1(angle):
    pwm.channels[12].duty_cycle = AnglesToPWM(angle, 620, 230, 180, 35)

def Calf_2(angle):
    pwm.channels[3].duty_cycle = AnglesToPWM(angle, 140, 530, 180, 35)

def Calf_3(angle):
    pwm.channels[0].duty_cycle = AnglesToPWM(angle, 620, 230, 180, 35)

def Calf_4(angle):
    pwm.channels[15].duty_cycle = AnglesToPWM(angle, 140, 530, 180, 35)

# High-level functions for joint, thigh, and calf control
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

# Robot setup and configuration functions
def LegsUp():
    Joints(90)
    Thighs(180)
    Calfs(180)

def SetUp():
    print("Assemble the servo horns")
    Joints(90)
    Thighs(90)
    Calfs(180)

# Function to control robot movements
def Sit():
    Calfs(30)  # Legs touching the ground
    Thighs(170)

def StandUp():
    LegsUp()
    Calfs(30)
    Thighs(170)
    Thighs(135)

def ShiftTo(position):
    # Adjust position
    Xposition()
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
        Xposition()

# Functions for high-level movements
def Forward():
    # Walking forward
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

# Functions for walking backward
def Backward():
    Calf_4(45)
    Calf_1(45)
    Thigh_1(160)
    Thigh_3(160)
    Joint_1(90)
    Joint_3(90)
    Joint_2(60)
    Joint_4(120)
    Thigh_1(135)
    Thigh_3(135)
    Thigh_2(160)
    Thigh_4(160)
    Joint_2(90)
    Joint_4(90)
    Joint_1(60)
    Joint_3(120)
    Thigh_2(135)
    Thigh_4(135)

# Rotations
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

# Movement in crab-walking style
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

# Simple hello gesture
def Hi():
    ShiftTo(3)
    Thigh_1(170)
    for _ in range(5):
        Calf_1(90)
        time.sleep(0.2)
        Calf_1(160)
    Calf_1(50)
    Xposition()

# Quick shuffle
def Shuffle():
    ShiftTo(1)
    ShiftTo(2)
    ShiftTo(3)
    ShiftTo(4)
    Xposition()

# Different walking patterns
def LegPositionFB(y, leg, step):
    # Creep gait positioning
    c = 93
    T = 75
    h = 40
    Xs = 31.8
    Z = math.sqrt(y**2 + Xs**2)
    w = math.sqrt(h**2 + Z**2)
    thetaH = math.degrees(math.atan(Z/h))
    thetaJ = 135 - math.degrees(math.atan(y/Xs))
    thetaT = math.degrees(math.acos((T**2 + w**2 - c**2) / (2 * T * w))) + thetaH
    thetaC = math.degrees(math.acos((T**2 + c**2 - w**2) / (2 * T * c)))

    if step == 0:
        if leg == 1:
            Thigh_1(175)
            Joint_2(50)
        elif leg == 2:
            Thigh_2(175)
            Joint_1(50)
        elif leg == 3:
            Thigh_3(175)
            Joint_4(50)
        elif leg == 4:
            Thigh_4(175)
            Joint_3(50)

    # Adjust leg positions
    if leg == 1:
        Joint_1(thetaJ)
        Thigh_1(thetaT)
        Calf_1(thetaC)
        Thigh_3(135)
        Joint_2(90)
    elif leg == 2:
        Joint_2(thetaJ)
        Thigh_2(thetaT)
        Calf_2(thetaC)
        Thigh_4(135)
        Joint_1(90)
    elif leg == 3:
        Joint_3(thetaJ)
        Thigh_3(thetaT)
        Calf_3(thetaC)
        Thigh_1(135)
        Joint_4(90)
    elif leg == 4:
        Joint_4(thetaJ)
        Thigh_4(thetaT)
        Calf_4(thetaC)
        Thigh_2(135)
        Joint_3(90)

def C_F():
    # Creep gait forward
    LegPositionFB(80, 1, 0)
    LegPositionFB(31.8, 1, 1)
    LegPositionFB(31.8, 4, 1)
    LegPositionFB(1, 2, 1)
    LegPositionFB(80, 3, 1)
    LegPositionFB(1, 3, 0)

    LegPositionFB(80, 2, 0)
    LegPositionFB(31.8, 2, 1)
    LegPositionFB(1, 1, 1)
    LegPositionFB(31.8, 3, 1)
    LegPositionFB(80, 4, 1)
    LegPositionFB(1, 4, 0)

def C_B():
    # Creep gait backward
    LegPositionFB(80, 3, 0)
    LegPositionFB(31.8, 2, 1)
    LegPositionFB(31.8, 3, 1)
    LegPositionFB(1, 4, 1)
    LegPositionFB(80, 1, 1)
    LegPositionFB(1, 1, 0)

    LegPositionFB(80, 4, 0)
    LegPositionFB(31.8, 4, 1)
    LegPositionFB(1, 3, 1)
    LegPositionFB(31.8, 1, 1)
    LegPositionFB(80, 2, 1)
    LegPositionFB(1, 2, 0)

# Uncomment this line only when ready to run robot control code
SetUp()
