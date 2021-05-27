"""
All of our prototype robot's utility functions are included in here
This way, the main.py file can stay clean and be modified easily.
"""

from time import sleep

def line_distance_mm(adc1, adc2, adc3, adc4):
    """
    Ultrasonic sensor reading function that averages the result of the
    past numReadings. Returns distance in mm of object in front of
    ultrasonic sensor.
    :param adc1: ADC input for the left-most (from head-on) line sensor
    :param adc2: ADC input for the centre-left (from head-on) line sensor
    :param adc3: ADC input for the centre-right (from head-on) line sensor
    :param adc4: ADC input for the right-most (from head-on) line sensor
    :return: Distance in mm of object in front of ultrasonic sensor
    """
    # These values contain the offset of the line sensors relative to the centre of the vehicle.
    x1 = -22
    x2 = -8
    x3 = 7
    x4 = 20

    # Read our sensor data
    w1 = adc1.read()
    w2 = adc2.read()
    w3 = adc3.read()
    w4 = adc4.read()

    num = w1*x1 + w2*x2 + w3*x3 + w4*x4
    denom = w1 + w2 + w3 + w4

    # line_dist contains the offset of the line relative to the centre of the vehicle
    dist_mm = num/denom

    return dist_mm


# Globals for ultrasonic_read()
ultraRunOnce = 0
ultraReadArray = []
ultraReadIndex = 0
ultraReadTotal = 0

def ultrasonic_read(ultraSens, numReadings):
    """
    Ultrasonic sensor reading function that averages the result of the
    past numReadings. Returns distance in mm of object in front of
    ultrasonic sensor.
    :param ultraSens: Ultrasonic sensor object from ultrasonic.py
    :param numReadings: Number of readings to take and average
    :return: Distance in mm of object in front of ultrasonic sensor
    """
    global ultraRunOnce
    global ultraReadArray
    global ultraReadIndex
    global ultraReadTotal

    if ultraRunOnce == 0:
        ultraReadArray = [0]*numReadings
        ultraRunOnce = 1

    ultraReadTotal -= ultraReadArray[ultraReadIndex]
    ultraReadArray[ultraReadIndex] = ultraSens.distance_mm()
    ultraReadTotal += ultraReadArray[ultraReadIndex]
    ultraReadIndex += 1

    if ultraReadIndex >= numReadings:
        ultraReadIndex = 0

    readAvg = ultraReadTotal / numReadings
    sleep(0.1)

    return readAvg

left_timer = 0
right_timer = 0

def straight_compensate(side, enc, mod=1.0):
    global left_timer
    global right_timer
    left_clicks = enc.get_left()
    right_clicks = enc.get_right()
    if side == "right":
        if left_clicks > right_clicks:
            left_timer += 1
        else: left_timer = 0
        return left_timer * mod
    elif side == "left":
        if right_clicks > left_clicks:
            right_timer += 1
        else: right_timer = 0
        return right_timer * mod
    else:
        left_timer = 0
        right_timer = 0


def motor_calibration(motor_left, motor_right, enc):
    """
    This function generates lines of CSV in the REPL output representing the:
        Current PWM value
        Encoder count for left wheel
        Encoder count for right wheel
    In three respective columns. This should be copy-pasted into a file and
    parsed with the motor_calibration_plot() function.
    :param motor_left: Motor object from motor.py, corresponding to left wheel
    :param motor_right: Motor object from motor.py, corresponding to right wheel
    :param enc: Encoder object from encode.py
    """
    print("PWM, ENC_L, ENC_R")
    for testPWM in range(0, 101, 5):  # Loop from 0-100% increasing by 5%
        enc.clear_count()
        # Drive forwards for 1 second at specified pwm
        motor_left.ctrl_alloc(testPWM)
        motor_right.ctrl_alloc(testPWM)
        sleep(1)

        # Wait for 1 second, print encoder counts in CSV format
        motor_left.ctrl_alloc(0)
        motor_right.ctrl_alloc(0)
        left_count = enc.get_left()
        right_count = enc.get_right()
        print("{:3d}, {:4d}, {:4d}".format(testPWM, left_count, right_count))
        sleep(1)
    motor_left.ctrl_alloc(0)
    motor_right.ctrl_alloc(0)
    print("Measurements generated! Paste the above inside motor_csv.txt")


def apds9960_distance_calibration(apds9960, ultra):
    """
    Generates lines of CSV in the REPL output representing the:
        Proximity of the APDS9960 RGB Sensor, and
        Proximity of the ultrasonic sensor in mm
    In two respective columns. This should be copy-pasted into a file
    :param apds9960: APDS9960 sensor object from APDS9960LITE.py
    :param ultra: Ultrasonic object from ultrasonic.py
    """
    for i in range(100):  # Take 100 measurements
        proximity_measurement = apds9960.prox.proximityLevel  # Read the proximity
        ultrasonic_measurement_mm = ultrasonic_read(ultra, 10)
        print("{:3d}, {:4.2f}".format(proximity_measurement, ultrasonic_measurement_mm))
        sleep(0.2)  # Wait for measurement to be ready
    print("Measurements generated! Paste the above inside rgb_csv.txt")
