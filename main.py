
# The following are a bunch of variables which enable/disable different robot functionality.
enableUltrasonic       = 1
enableMotorCalibration = 0
enableLineSensors      = 1
enableLineFollowing    = 1

from time import sleep
from machine import Pin
from pyb import ADC
from pyb import Pin as APin
from motor import Motor
from encoder import Encoder
from util import motor_calibration, line_sensor_read, ultrasonic_read
import sys

if enableLineSensors:
    lnSens_A1 = ADC(APin("A1"))
    lnSens_A2 = ADC(APin("A2"))
    lnSens_A3 = ADC(APin("A3"))
    lnSens_A4 = ADC(APin("A4"))

if enableUltrasonic:
    import ultrasonic
    # D13 is trigger, D12 is echo, and our timeout is set to 1.0 metres
    ultraSens = ultrasonic.HCSR04("D13", "D12", echo_timeout_us=1000*2/(340.29*1e-3))

# Create left and right `Motor' objects
motor_left = Motor("left", "D6", "D7", "D4")
motor_right = Motor("right", "D8", "D9", "D5")

# Create encoder object
enc = Encoder("D2", "D3") # D2 is left wheel, D3 is right wheel

fwdSpeed = 65  # Forward and backward speeds for motors, respectively
bckSpeed = 65

while True:
    motor_left.ctrl_alloc(1, 0)   # Set the left motor to run forwards, speed 0
    motor_right.ctrl_alloc(1, 0)  # Set the right motor to run forwards, speed 0
    enc.clear_count()  # Reset the encoder to zero

    if enableMotorCalibration:
        motor_calibration(motor_left, motor_right, enc)
        sys.exit(0)

    if enableLineSensors:
        line_dist = line_sensor_read(lnSens_A1, lnSens_A2, lnSens_A3, lnSens_A4)
    else:
        enableLineFollowing = 0
        line_dist = 0

    if enableUltrasonic:
        ultra_dist = ultrasonic_read(ultraSens, 10)
        if ultra_dist > 160:
            motor_left.ctrl_alloc(1, fwdSpeed)  # Forwards
            motor_right.ctrl_alloc(1, fwdSpeed)
        elif ultra_dist < 120:
            motor_left.ctrl_alloc(0, bckSpeed)  # Backwards
            motor_right.ctrl_alloc(0, bckSpeed)
