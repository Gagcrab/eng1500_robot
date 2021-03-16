from time import sleep
from motor import Motor
from machine import Pin

# Print to REPL at start of program
print("Hello, world!")

# Create left and right `Motor' objects
motor_left = Motor("left", "D6", "D7", "D4")
motor_right = Motor("right", "D8", "D9", "D5")

# Create line sensor object
line_sensor = Pin("A0", Pin.IN)

# Variables
motor_speed = 0

while True:
    motor_left.set_forwards()
    motor_right.set_forwards()
    sensor_val = line_sensor.value()

#    if sensor_val == 0:  # if obstacle detected
#        motor_speed = 0
#    else:
#        motor_speed = 75

    motor_left.duty(sensor_val*60)
    motor_right.duty(sensor_val*30)
    sleep(0.1)