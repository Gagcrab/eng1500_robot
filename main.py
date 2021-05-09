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
ENC_L = "D2"; ENC_R = "D3" ; ULTRA_TIMEOUT = 1000*2/(340.29*1e-3) # 1 m
MOTOR_L_IN1    = "D6" ;      MOTOR_R_IN1    = "D8"
MOTOR_L_IN2    = "D7" ;      MOTOR_R_IN2    = "D9"
MOTOR_L_EN_PWM = "D4" ;      MOTOR_R_EN_PWM = "D5"

# Motor speeds and limit (percentages)
fwdSpeed = 70 ;              bckSpeed = -65
motorLimit = 90

# Sensitivity modifier constant of front line sensors
lineSensMod = 1.0  # 1.0 = no effect

# Sensitivity modifier constant of left and right IR sensors
sideCloseThres = 50  # 50 mm
sideStopThres = 20   # 20 mm
sideSensMod = 1.0    # 1.0 = no effect. Higher values increase correction aggressiveness.

# APDS9960 proximity sensor
proxStopThres = 50  # 50 mm

# Threshold of ultrasonic sensor before changing to STOP_CHECK
ultraStopThres = 100  # 100 mm

# If both side IR sensors and the ultrasonic are < garageThres,
# then assume we are inside the garage
garageThres = 100  # 100 mm

# List of states, and default state
state_list = ['DRIVE_FORWARD', 'DRIVE_BACKWARD', 'STOP_CHECK',]
state = "STOP_CHECK"

# Enable/disable different robot functionality.
enableUltrasonic       = 1 ; enableProximity        = 1
enableLineSensors      = 1 ; enableMotorCalibration = 0
enableLineFollowing    = 0 ; enableRGBCalibration   = 0
enableMovement         = 0 ; enableSideIRSensors    = 0
"""
=======================================================================
------------------------ Initialisation code --------------------------
=======================================================================
"""
if enableRGBCalibration:  # RGB calibration requires prox and Ultrasonic
    enableProximity  = 1
    enableUltrasonic = 1

# From front on perspective, sensor 1 is leftmost, sensor 4 is rightmost
if enableLineSensors:
    lnSens_A1 = ADC(APin(LN_SENS_1))
    lnSens_A2 = ADC(APin(LN_SENS_2))
    lnSens_A3 = ADC(APin(LN_SENS_3))
    lnSens_A4 = ADC(APin(LN_SENS_4))
else:
    enableLineFollowing = 0

if enableSideIRSensors:
    # TODO once mounted
    print("asdf")

if enableProximity:
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
enc = Encoder(ENC_L, ENC_R)

if enableMovement == 0:
    fwdSpeed = 0
    bckSpeed = 0

sleep(0.2)  # Wait to ensure everything is initialized

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
line_dist = 0  # Variables to store sensor values, all in mm
side_l_dist = 0
side_r_dist = 0
ultra_dist = 0
prox_dist = 0

while True:
    motor_left.ctrl_alloc(0)    # Set left motor to run forwards, speed 0
    motor_right.ctrl_alloc(0)   # Set right motor to run forwards, speed 0
    enc.clear_count()           # Reset the encoder to zero

    if enableLineSensors:
        line_dist = line_distance_mm(lnSens_A1, lnSens_A2, lnSens_A3, lnSens_A4)

    if enableSideIRSensors:
        # TODO once mounted
        print("asdf")

    if enableProximity:
        # TODO: replace line with mm conversion
        prox_dist = apds9960.prox.proximityLevel  # Read the proximity value
        #print(proximity_measurement)

    if enableUltrasonic:  # ultrasonic_read() causes a delay of 100ms in the loop
        ultra_dist = ultrasonic_read(ultraSens, 10)

    motorChangeL = 0
    motorChangeR = 0
    """
    ===================================================================
    ----------------------------- States ------------------------------
    ===================================================================
    """
    """
    STOP_CHECK is our "hub state" of sorts. It is used as a transition between
    every other state, in order to simplify things. We should only ever be in
    this state for one loop iteration!
    """
    if state == "STOP_CHECK":
        motorChangeL = 0 ; motorChangeR = 0
        fwdConds     = 3 ; fwdCondsMet  = 0  # Number of condition we have to
        revConds     = 3 ; revCondsMet  = 0  # meet in order to change state

        # Transition conditions
        if enableProximity:
            if prox_dist > proxStopThres:
                fwdCondsMet += 1
            elif prox_dist < proxStopThres:  # Too close; reverse
                revCondsMet += 1
        else:  # If APDS9960 is disabled, we don't need to meet this condition
            fwdConds -= 1 ; revConds -= 1

        if enableSideIRSensors:
            if side_l_dist > sideStopThres and side_r_dist > sideStopThres:
                fwdCondsMet += 1
            elif side_l_dist < sideStopThres or side_r_dist < sideStopThres:
                revCondsMet += 1
        else:  # if side sensors disabled, we don't need to meet this condition
            fwdConds -= 1 ; revConds -= 1

        if enableUltrasonic:
            if ultra_dist > ultraStopThres:
                fwdCondsMet += 1
            elif ultra_dist < ultraStopThres:  # Too close; reverse
                revCondsMet += 1
        else:  # If ultrasonic disabled, we don't need to meet this condition
            fwdConds -= 1 ; revConds -= 1

        if fwdCondsMet == fwdConds:  # Check if we meet all conditions
            state = "DRIVE_FORWARD"
        elif revCondsMet == revConds:
            state = "DRIVE_REVERSE"
        else:                        # This is bad!
            print("Neither DRIVE_FORWARD or DRIVE_REVERSE conditions met!")
            state = state
        ### END TRANSITION CONDITIONS ###
        sleep(0.5)  # Wait before switching to another state

    elif state == "DRIVE_FORWARD":
        motorChangeL = fwdSpeed
        motorChangeR = fwdSpeed

        # Transition conditions
        if enableProximity:
            if prox_dist < proxStopThres:
                state = "STOP_CHECK"
        elif enableUltrasonic:
            if ultra_dist < ultraStopThres:  # Within safety threshold
                state = "STOP_CHECK"
        elif enableSideIRSensors:
            if side_l_dist < sideStopThres or side_r_dist < sideStopThres:
                # Super close! Emergency stop
                state = "STOP_CHECK"
        else:
            state = state
        ### END TRANSITION CONDITIONS ###

        if enableLineFollowing:
            if line_dist > 0:  # Veering left of the line, slow right motor
                motorChangeR -= abs(line_dist)/lineSensMod
            else:              # Veering right of the line, slow left motor
                motorChangeL -= abs(line_dist)/lineSensMod

        if enableSideIRSensors:
            if side_l_dist < sideCloseThres:    # Left side of vehicle within safe dist
                motorChangeR = 0              # Stop right motor to be safe
            elif side_r_dist < sideCloseThres:  # Right side of vehicle within safe dist
                motorChangeL = 0              # Stop left motor to be safe

    elif state == "DRIVE_REVERSE":
        motorChangeL = bckSpeed
        motorChangeR = bckSpeed

        # Transition conditions
        # TODO: not done yet
        ### END TRANSITION CONDITIONS ###

    else:
        print("State machine has fucked up, blame Isaac")

    if motorChangeL > motorLimit:  # Don't fuckin' book it
        motorChangeL = motorLimit
    if motorChangeR > motorLimit:
        motorChangeR = motorLimit

    print(state)
    motor_left.ctrl_alloc(motorChangeL)  # Change our motor speeds
    motor_right.ctrl_alloc(motorChangeR)
    sleep(0.02)
