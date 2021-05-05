import matplotlib.pyplot as plt
import csv


def motor_calibration_plot(filename):
    """
    Reads CSV data with the follow three respective columns:
        Current PWM value
        Encoder count for left wheel
        Encoder count for right wheel
    Two curves will be generated demonstrating the discrepancy between
    the duty cycles of our motors.
    :param filename: Name of file containing CSV data to plot
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


def apds9960_calibration_plot(filename):
    """
    Reads CSV data with the follow three respective columns:
        APDS9960 distance sensor value
        Ultrasonic distance measurement in mm
    The generated curve provides of a comparison between the ultrasonic
    distance measurement (in mm) and the arbitrary APDS9960 distance value.
    :param filename: Name of file containing CSV data to plot
    """
    apds9960 = []
    ultrasonic = []

    with open(filename, 'r') as csvfile:
        plotdata = csv.reader(csvfile, delimiter=',')
        for row in plotdata:
            apds9960.append(float(row[0]))
            ultrasonic.append(float(row[1]))

    plt.plot(ultrasonic,apds9960,marker='o')
    plt.title("APDS9960 vs Ultrasonic Sensor")
    plt.xlabel("Ultrasonic distance [mm]")
    plt.ylabel("APDS9960 distance [no units]")
    plt.xlim(0, 20)
    plt.show()


# Uncomment whichever function you wish to call below
#motor_calibration_plot('motor_csv.txt')
apds9960_calibration_plot('distance_csv.txt')