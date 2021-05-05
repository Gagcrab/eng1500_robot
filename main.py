from time import sleep
from machine import I2C, Pin
from pyb import ADC
from pyb import Pin as APin
from motor import Motor
from encoder import Encoder
from APDS9960LITE import APDS9960LITE
from util import motor_calibration, apds9960_distance_calibration, \
                 line_distance_mm, ultrasonic_read
import sys
import ultrasonic

"""
=======================================================================
------------------- Pin configuration and constants -------------------
=======================================================================
"""
LN_SENS_1 = "A1" ;           RGB_1 = "PB13"
LN_SENS_2 = "A2" ;           RGB_2 = "PB14"
LN_SENS_3 = "A3" ;           ULTRA_TRIG = "D13"
LN_SENS_4 = "A4" ;           ULTRA_ECHO = "D12"
ENC_L = "D2"; ENC_R = "D3" ; ULTRA_TIMEOUT = 1000*2/(340.29*1e-3) # 1.0 metres
MOTOR_L_IN1    = "D6" ;      MOTOR_R_IN1    = "D8"
MOTOR_L_IN2    = "D7" ;      MOTOR_R_IN2    = "D9"
MOTOR_L_EN_PWM = "D4" ;      MOTOR_R_EN_PWM = "D5"
fwdSpeed = 75 ;              bckSpeed = -65
STATE = "DRIVE_FORWARD"  # Specify default state here

# Enable/disable different robot functionality.
enableUltrasonic       = 1 ; enableRGB              = 1
enableLineSensors      = 1 ; enableMotorCalibration = 0
enableLineFollowing    = 0 ; enableRGBCalibration   = 0
enableMovement         = 0
"""
=======================================================================
------------------------ Initialisation code --------------------------
=======================================================================
"""
if enableRGBCalibration:    # RGB calibration requires RGB and Ultrasonic functionality
    enableRGB        = 1
    enableUltrasonic = 1

# From front on perspective, sensor 1 is leftmost, sensor 4 is rightmost
if enableLineSensors:
    lnSens_A1 = ADC(APin(LN_SENS_1))
    lnSens_A2 = ADC(APin(LN_SENS_2))
    lnSens_A3 = ADC(APin(LN_SENS_3))
    lnSens_A4 = ADC(APin(LN_SENS_4))

if enableRGB:
    # Initialise I2C bus
    i2c = I2C(scl=Pin(RGB_1), sda=Pin(RGB_2))
    # Initialise APDS9960
    apds9960 = APDS9960LITE(i2c)    # Create APDS9960 sensor object
    apds9960.prox.enableSensor()    # Send I2C command to enable sensor

if enableUltrasonic:
    ultraSens = ultrasonic.HCSR04(ULTRA_TRIG, ULTRA_ECHO, echo_timeout_us=ULTRA_TIMEOUT)

# Create left/right Motor, Encoder objects
motor_left = Motor("left", MOTOR_L_IN1, MOTOR_L_IN2, MOTOR_L_EN_PWM)
motor_right = Motor("right", MOTOR_R_IN1, MOTOR_R_IN2, MOTOR_R_EN_PWM)
enc = Encoder(ENC_L, ENC_R)  # D2 is left wheel, D3 is right wheel

if enableMovement == 0:
    fwdSpeed = 0
    bckSpeed = 0

sleep(0.2)     # Wait to ensure everything is initialized

if enableMotorCalibration:
    motor_left.ctrl_alloc(0)
    motor_right.ctrl_alloc(0)
    enc.clear_count()
    motor_calibration(motor_left, motor_right, enc)
    sys.exit(0)

if enableRGBCalibration:
    apds9960_distance_calibration(apds9960, ultraSens)
    sys.exit(0)
"""
=======================================================================
----------------------- Main loop begins below ------------------------
=======================================================================
"""
while True:
    motor_left.ctrl_alloc(0)    # Set left motor to run forwards, speed 0
    motor_right.ctrl_alloc(0)   # Set right motor to run forwards, speed 0
    enc.clear_count()           # Reset the encoder to zero

    if enableLineSensors:
        line_dist = line_distance_mm(lnSens_A1, lnSens_A2, lnSens_A3, lnSens_A4)
        if enableLineFollowing:
            if line_dist > 0:
                motor_left.ctrl_alloc(fwdSpeed)
                motor_right.ctrl_alloc(fwdSpeed - abs(line_dist)/1.5)
            else:
                motor_left.ctrl_alloc(fwdSpeed - abs(line_dist)/1.5)
                motor_right.ctrl_alloc(fwdSpeed)
    else:
        enableLineFollowing = 0
        line_dist = 0

    if enableRGB:
        proximity_measurement = apds9960.prox.proximityLevel  # Read the proximity value
        print(proximity_measurement)  # Print proximity value
        sleep(0.2)  # Wait for measurement to be ready

    if enableUltrasonic:
        ultra_dist = ultrasonic_read(ultraSens, 10)
        if ultra_dist > 160:
            motor_left.ctrl_alloc(fwdSpeed)  # Forwards
            motor_right.ctrl_alloc(fwdSpeed)
        elif ultra_dist < 120:
            motor_left.ctrl_alloc(bckSpeed)  # Backwards
            motor_right.ctrl_alloc(bckSpeed)

    sleep(0.1)
