from time import sleep
from machine import I2C, Pin
from pyb import ADC, Servo, Pin as APin
from motor import Motor
from encoder import Encoder
from APDS9960LITE import APDS9960LITE
from util import motor_calibration, apds9960_distance_calibration, \
                 line_distance_mm, ultrasonic_read, proximity_read
import sys, ultrasonic, ssd1306

# Enable/disable different robot functionality.
enableUltrasonic       = 1; enableProximity        = 1
enableLineSensors      = 1; enableSideIRSensors    = 1
enableMovement         = 0; enableDisplay          = 1
# Only use these for generating calibration data
enableRGBCalibration   = 0; enableMotorCalibration = 0

"""
=======================================================================
------------------- Pin configuration and constants -------------------
=======================================================================
"""
LN_SENS_1 = "A4";           RGB_1 = "PB13"
LN_SENS_2 = "A3";           RGB_2 = "PB14"
LN_SENS_3 = "A2";           ULTRA_TRIG = "D13"
LN_SENS_4 = "A1";           ULTRA_ECHO = "D12"
ENC_L = "D2"; ENC_R = "D3"; ULTRA_TIMEOUT = 1000*2/(340.29*1e-3) # 1 m
MOTOR_L_IN1    = "D6";      MOTOR_R_IN1    = "D8"
MOTOR_L_IN2    = "D7";      MOTOR_R_IN2    = "D9"
MOTOR_L_EN_PWM = "D4";      MOTOR_R_EN_PWM = "D5"
SIDE_SENS_L = "D11";        OLED_SCL = "PB1"
SIDE_SENS_R = "D10";        OLED_SDA = "PB2"

# Motor speeds and limit (PWM percentages)
fwdSpeed = 70 ;              bckSpeed = -65
motorLimit = 90

# Sensitivity modifier constants. Higher values increase correction aggressiveness.
lineSensMod = 1.0  # 1.0 = no effect

# Distance thresholds for the state machine
proxCloseThres = 2.0  # about 120 mm
proxStopThres  = 8.0  # about 60 mm
ultraStopThres = 100  # 100 mm
garageThres = 100     # 100 mm

# Default state
state = "STOP_CHECK"

"""
=======================================================================
------------------------ Initialisation code --------------------------
=======================================================================
"""
# OLED display
if enableDisplay:
    oled_i2c = I2C(-1, scl=Pin(OLED_SCL), sda=Pin(OLED_SDA))
    oled = ssd1306.SSD1306_I2C(128, 32, oled_i2c)
    print("OLED display initialised.")

if enableRGBCalibration:  # RGB calibration requires prox and Ultrasonic
    enableProximity  = 1
    enableUltrasonic = 1

if enableLineSensors:
    lnSens_A1 = ADC(APin(LN_SENS_1))
    lnSens_A2 = ADC(APin(LN_SENS_2))
    lnSens_A3 = ADC(APin(LN_SENS_3))
    lnSens_A4 = ADC(APin(LN_SENS_4))
    print("Line sensors initialised.")

if enableSideIRSensors:
    sideSens_L = Pin(SIDE_SENS_L, Pin.IN)
    print("Left side sensor initialised.")
    sideSens_R = Pin(SIDE_SENS_R, Pin.IN)
    print("Right side sensor initialised.")

if enableProximity:
    # Initialise I2C bus
    prox_i2c = I2C(-1, scl=Pin(RGB_1), sda=Pin(RGB_2))
    # Initialise APDS9960
    apds9960 = APDS9960LITE(prox_i2c)  # Create APDS9960 sensor object
    apds9960.prox.enableSensor()       # Send I2C command to enable sensor
    print("Proximity sensor initialised.")

if enableUltrasonic:
    ultraSens = ultrasonic.HCSR04(ULTRA_TRIG, ULTRA_ECHO, echo_timeout_us=ULTRA_TIMEOUT)
    my_servo = Servo(1)
    my_servo.calibration(640, 2420, 1700, 2470, 2220)
    my_servo.angle(0)   # Aim ultrasonic straight
    print("Ultrasonic sensor and servo initialised.")

# Create left/right Motor, Encoder objects
motor_left = Motor("left", MOTOR_L_IN1, MOTOR_L_IN2, MOTOR_L_EN_PWM)
motor_right = Motor("right", MOTOR_R_IN1, MOTOR_R_IN2, MOTOR_R_EN_PWM)
enc = Encoder(ENC_L, ENC_R)
print("Encoder initialised.")

if enableMovement == 0:
    fwdSpeed = 0
    bckSpeed = 0

print("Initialisation complete!")
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
line_dist = 0  # Variables to store sensor values
side_l_dist = 0
side_r_dist = 0
ultra_dist = 0
prox_dist = 0
runOnce = 0

while True:
    motor_left.ctrl_alloc(0)    # Set left motor to run forwards, speed 0
    motor_right.ctrl_alloc(0)   # Set right motor to run forwards, speed 0
    enc.clear_count()           # Reset the encoder to zero
    oled.fill(0)                # Clear OLED screen

    if enableLineSensors:
        line_dist = line_distance_mm(lnSens_A1, lnSens_A2, lnSens_A3, lnSens_A4)
        lineStr = 'Line:{:.1f}'.format(float(line_dist))
    else: lineStr = 'Line:n/a'

    if enableSideIRSensors:
        side_l_dist = sideSens_L.value()
        sideSens_L_Str = 'iL:{}'.format(int(side_l_dist))
        side_r_dist = sideSens_R.value()
        sideSens_R_Str = 'iR:{}'.format(int(side_r_dist))
    else: sideSens_L_Str = 'iL:n/a'; sideSens_R_Str = 'iR:n/a'

    if enableProximity:
        prox_dist = proximity_read(apds9960, 10)
        proxStr = 'Prox:{:.1f}'.format(float(prox_dist))
    else: proxStr = 'Prox:n/a'

    if enableUltrasonic:  # ultrasonic_read() causes a delay of 100ms in the loop
        ultra_dist = ultrasonic_read(ultraSens, 10)
        if ultra_dist < 0:
            ultra_dist = 0
        ultraStr = 'uF:{}'.format(int(ultra_dist))
    else: ultraStr = 'Ultra:n/a'

    # We will need to wait till later to get ultra_l_dist and ultra_r_dist!

    stateStr = 'State: ' + str(state)

    motorChangeL = 0
    motorChangeR = 0
    """
    ===================================================================
    ----------------------------- States ------------------------------
    ===================================================================
    """


    # Stop and check
    if state == "STOP_CHECK":
        motorChangeL = 0
        motorChangeR = 0
        pTooClose = 0
        iLeftTooClose = 0
        iRightTooClose = 0
        uTooClose = 0
        uLeftTooClose = 0
        uRightTooClose = 0
        tooCloseLeft = 0
        tooCloseRight = 0

        if enableProximity:
            if prox_dist < proxStopThres:
                pTooClose = 1

        if enableSideIRSensors:
            if side_l_dist == 0:
                iLeftTooClose = 1
            if side_r_dist == 0:
                iRightTooClose = 1

        if enableUltrasonic:
            # Turn ultrasonic right and gather 10 readings. This is required
            # because ultrasonic_read() takes the average of the past N readings,
            # and this average will be affected by data from previous readings
            # unless we run enough iterations to replace it all.
            my_servo.angle(-90, 1000)  # Turn servo right
            sleep(0.5)
            i = 0
            while i < 10:
                ultra_l_dist = ultrasonic_read(ultraSens, 10)
                i += 1
            ultra_L_Str = '{}'.format(int(ultra_l_dist))

            # Turn ultrasonic left and gather 10 readings
            my_servo.angle(90, 1000)  # Turn servo left
            sleep(0.5)
            j = 0
            while j < 10:
                ultra_r_dist = ultrasonic_read(ultraSens, 10)
                j += 1
            ultra_R_Str = '{}'.format(int(ultra_r_dist))

            # We have to redo the front facing ultrasonic reading, in order
            # to repopulate the array
            my_servo.angle(0)   # Turn servo straight again
            sleep(0.5)
            k = 0
            while k < 10:
                ultra_dist = ultrasonic_read(ultraSens, 10)
                k += 1
            ultraStr = 'uF:{}'.format(int(ultra_dist))

            if ultra_l_dist < ultraStopThres:
                uLeftTooClose = 1
            if ultra_r_dist < ultraStopThres:
                uRightTooClose = 1
            if ultra_dist < ultraStopThres:
                uTooClose = 1

        # Transition conditions
        if pTooClose == 0:      # Does proxy say we're too close?
            if uTooClose == 0:      # Nope, but does ultra say we're too close?
                direction = 1           # Nope! we're going forward
            else: direction = -1        # Yep, we're reversing
        else: direction = -1        # Yep, we're reversing

        if iLeftTooClose:
            tooCloseLeft = 1
        elif iRightTooClose:
            tooCloseRight = 1
        else:
            if uLeftTooClose:
                tooCloseLeft = 1
            elif iRightTooClose:
                tooCloseRight = 1

        if direction == 1:
            if tooCloseLeft:
                state = "FWD_CLOSE_L"
            elif tooCloseRight:
                state = "FWD_CLOSE_R"
            else:
                state = "FWD_CLEAR"
        elif direction == -1:
            if tooCloseLeft:
                state = "REV_CLOSE_L"
            elif tooCloseRight:
                state = "REV_CLOSE_R"
            else:
                state = "REV_CLEAR"
        else: state = state
        ### END TRANSITION CONDITIONS ###



    # Everything good, drive forward
    elif state == "FWD_CLEAR":
        motorChangeL = fwdSpeed
        motorChangeR = fwdSpeed


        # Transition conditions
        if enableProximity and prox_dist < proxStopThres or \
           enableUltrasonic and ultra_dist < ultraStopThres or \
           enableSideIRSensors and side_l_dist == 0 or \
           enableSideIRSensors and side_r_dist == 0:
                state = "STOP_CHECK"
        else: state = state
        ### END TRANSITION CONDITIONS ###


        if enableLineSensors:
            if line_dist > 0:  # Veering left of the line, slow right motor
                motorChangeR -= abs(line_dist)/lineSensMod
            else:              # Veering right of the line, slow left motor
                motorChangeL -= abs(line_dist)/lineSensMod



    # We have a wall on our left, move forward to evade
    elif state == "FWD_CLOSE_L":
        motorChangeL = fwdSpeed
        motorChangeR = fwdSpeed - 20

        # Transition conditions
        if enableProximity and prox_dist < proxStopThres or \
           enableUltrasonic and ultra_dist < ultraStopThres or \
           enableSideIRSensors and side_l_dist == 1:
                state = "STOP_CHECK"
        else: state = state
        ### END TRANSITION CONDITIONS ##



    # We have a wall on our right, move forward to evade
    elif state == "FWD_CLOSE_R":
        motorChangeL = fwdSpeed - 20
        motorChangeR = fwdSpeed

        # Transition conditions
        if enableProximity and prox_dist < proxStopThres or \
           enableUltrasonic and ultra_dist < ultraStopThres or \
           enableSideIRSensors and side_l_dist == 1:
                state = "STOP_CHECK"
        else: state = state
        ### END TRANSITION CONDITIONS ##



    # Something in front, back up
    elif state == "REV_CLEAR":
        motorChangeL = bckSpeed
        motorChangeR = bckSpeed


        # Transition conditions
        if enableProximity and prox_dist > proxStopThres or \
           enableUltrasonic and ultra_dist > ultraStopThres or \
           enableSideIRSensors and side_l_dist == 0 or \
           enableSideIRSensors and side_r_dist == 0:
                state = "STOP_CHECK"
        else: state = state
        ### END TRANSITION CONDITIONS ###


        if enableLineSensors:
            if line_dist > 0:  # Veering left of the line, slow left motor
                motorChangeL -= abs(line_dist)/lineSensMod
            else:              # Veering right of the line, slow right motor
                motorChangeR -= abs(line_dist)/lineSensMod


    # We have a wall on our left, reverse to evade
    elif state == "REV_CLOSE_L":
        motorChangeL = fwdSpeed - 20
        motorChangeR = fwdSpeed

        # Transition conditions
        if enableProximity and prox_dist > proxStopThres or \
           enableUltrasonic and ultra_dist > ultraStopThres or \
           enableSideIRSensors and side_r_dist == 0:
                state = "STOP_CHECK"
        else: state = state
        ### END TRANSITION CONDITIONS ###


    # We have a wall on our right, reverse to evade
    elif state == "REV_CLOSE_R":
        motorChangeL = fwdSpeed
        motorChangeR = fwdSpeed - 20

        # Transition conditions
        if enableProximity and prox_dist > proxStopThres or \
           enableUltrasonic and ultra_dist > ultraStopThres or \
           enableSideIRSensors and side_l_dist == 0:
                state = "STOP_CHECK"
        else: state = state
        ### END TRANSITION CONDITIONS ###

    else:
        print("State machine has fucked up, blame Isaac")

    if motorChangeL > motorLimit:  # Don't fuckin' book it
        motorChangeL = motorLimit
    if motorChangeR > motorLimit:
        motorChangeR = motorLimit

    print(lineStr)
    print("Left side sensor " + sideSens_L_Str)
    print("Right side sensor " + sideSens_R_Str)
    print(proxStr)
    print(ultraStr)
    print("uL:" + ultra_L_Str)
    print("uR:" + ultra_R_Str)
    print(stateStr)

    if enableDisplay:
        oled.text(lineStr, 0, 1); oled.text(sideSens_L_Str, 80, 1)
        oled.text(proxStr, 0, 9); oled.text(sideSens_R_Str, 80, 9)
        oled.text(ultraStr, 0, 17)
        oled.text(ultra_L_Str, 52, 17); oled.text(ultra_R_Str, 80, 17)
        oled.text(str(state), 0, 25)
        oled.show()

    if runOnce < 10:    # Don't do anything for first 10 iterations, to
        runOnce += 1    # allow ultrasonic/proximity arrays to populate
    else:
        motor_left.ctrl_alloc(motorChangeL)  # Change our motor speeds
        motor_right.ctrl_alloc(motorChangeR)

    sleep(0.02)
