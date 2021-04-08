"""
All of our prototype robot's utility functions are included in here
This way, the main.py file can stay clean and be modified easily.
"""

from time import sleep
import matplotlib.pyplot as plt
import csv


def line_sensor_read(adc_A1, adc_A2, adc_A3, adc_A4):
    # These values contain the offset of the line sensors relative to the centre of the vehicle.
    # TODO: Measure and tweak these values
    x1 = -22.5
    x2 = -7.5
    x3 = 7.5
    x4 = 22.5

    # Read our sensor data
    w1 = adc_A1.read()
    w2 = adc_A2.read()
    w3 = adc_A3.read()
    w4 = adc_A4.read()

    num = w1*x1 + w2*x2 + w3*x3 + w4*x4
    denom = w1 + w2 + w3 + w4

    # line_dist contains the offset of the line relative to the centre of the vehicle
    line_dist = num/denom

    print("Distance from line = {:3.2f}".format(line_dist))
    return line_dist


def ultrasonic_read(ultraSens, numReadings):
    numReadings = 10
    readings = [numReadings]
    readIndex = 0
    readTotal = 0
    readAvg = 0

    readTotal -= readings[readIndex]
    readings[readIndex] = ultraSens.distance_mm()
    readTotal += readings[readIndex]
    readIndex += 1

    if readIndex >= numReadings:
        readIndex = 0

    readAvg = readTotal / numReadings
    dist = readAvg
    print("Distance = {:6.2f} [mm]".format(dist))
    sleep(0.1)

    return dist


def motor_calibration(motor_left, motor_right, enc):
    """ This function generates lines of CSV in the REPL output representing the:
            Current PWM value
            Encoder count for left wheel
            Encoder count for right wheel
        In three respective columns. This should be copy-pasted into a file and
        parsed with the motor_calibration_plot() function.
    """
    print("PWM, ENC_L, ENC_R")
    for testPWM in range(0, 101, 5):  # Loop from 0-100% increasing by 5%
        enc.clear_count()
        # Drive forwards for 1 second at specified pwm
        motor_left.ctrl_alloc(1, testPWM)
        motor_right.ctrl_alloc(1, testPWM)
        sleep(1)

        # Wait for 1 second, print encoder counts in CSV format
        motor_left.ctrl_alloc(1, 0)
        motor_right.ctrl_alloc(1, 0)
        left_count = enc.get_left()
        right_count = enc.get_right()
        print("{:3d}, {:4d}, {:4d}".format(testPWM, left_count, right_count))
        sleep(1)


def motor_calibration_plot(filename):
    """ This function reads CSV data with the follow three respective columns:
            Current PWM value
            Encoder count for left wheel
            Encoder count for right wheel
        Two curves will be generated demonstrating the discrepancy between the
        duty cycles of our motors.
    """
    testPWM = []
    left_enc = []
    right_enc = []

    with open(filename, 'r') as csvfile:
        plotdata = csv.reader(csvfile, delimiter=',')
        for row in plotdata:
            testPWM.append(int(row[0]))
            left_enc.append(int(row[1]))
            right_enc.append(int(row[2]))

    plt.plot(testPWM, left_enc, marker='o')
    plt.plot(testPWM, right_enc, marker='o')
    plt.title("Marker PWM vs Encoder speed")
    plt.xlabel("PWM duty cycle")
    plt.ylabel("Encoder count / speed")
    plt.legend(['right motor', 'left motor'])
    plt.show()
