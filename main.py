from time import sleep
from machine import I2C, Pin
from pyb import ADC
from pyb import Pin as APin
from motor import Motor
from encoder import Encoder
from util import motor_calibration, apds9960_distance_calibration, \
                 line_distance_mm, ultrasonic_read
import sys

# The following are a bunch of variables which enable/disable different robot functionality.
enableUltrasonic       = 0
enableLineSensors      = 1
enableLineFollowing    = 0
enableRGB              = 1
enableMovement         = 0

# ONLY used for calibration/plotting
enableMotorCalibration = 1
enableRGBCalibration   = 0

if enableRGBCalibration:    # RGB calibration requires RGB and Ultrasonic functionality
    enableRGB        = 1
    enableUltrasonic = 1

# From a front on perspective, sensor 1 is leftmost, sensor 4 is rightmost
if enableLineSensors:
    lnSens_A1 = ADC(APin("A1"))
    lnSens_A2 = ADC(APin("A2"))
    lnSens_A3 = ADC(APin("A3"))
    lnSens_A4 = ADC(APin("A4"))

if enableRGB:
    from APDS9960LITE import APDS9960LITE
    # Initialise I2C bus
    i2c = I2C(scl=Pin("PB13"), sda=Pin("PB14"))
    # Initialise APDS9960
    apds9960 = APDS9960LITE(i2c)    # Create APDS9960 sensor object
    apds9960.prox.enableSensor()    # Send I2C command to enable sensor

if enableUltrasonic:
    import ultrasonic
    # D13 is trigger, D12 is echo, timeout is set to 1.0 metres
    ultraSens = ultrasonic.HCSR04("D13", "D12", echo_timeout_us=1000*2/(340.29*1e-3))

# Create left/right Motor, Encoder objects
motor_left = Motor("left", "D6", "D7", "D4")
motor_right = Motor("right", "D8", "D9", "D5")
enc = Encoder("D2", "D3")  # D2 is left wheel, D3 is right wheel

fwdSpeed = 75  # Forward and backward speeds for motors, respectively
bckSpeed = -65

sleep(0.2)     # Wait to ensure everything is initialized

if enableMovement == 0:
    fwdSpeed = 0
    bckSpeed = 0

while True:
    motor_left.ctrl_alloc(0)    # Set left motor to run forwards, speed 0
    motor_right.ctrl_alloc(0)   # Set right motor to run forwards, speed 0
    enc.clear_count()           # Reset the encoder to zero

    if enableMotorCalibration:
        from robot_plot import motor_calibration_plot
        filename = 'motor_csv.txt'
        motor_calibration(motor_left, motor_right, enc)
        input(f'Press enter once {filename} file is generated to show plot')
        motor_calibration_plot(filename)
        sys.exit(0)

    if enableRGBCalibration:
        from robot_plot import apds9960_calibration
        filename = 'rgb_csv.txt'
        apds9960_distance_calibration(apds9960, ultraSens)
        input(f'Press enter once {filename} file is generated to show plot')
        apds9960_calibration(filename)
        sys.exit(0)

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
