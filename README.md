# Source code for ENGG1500 robot

This is the Python code and modules for the autonomous vehicle I built in the first semester of my Computer Systems Engineering degree at the University of Newcastle. While the project was intended to be constructed in groups of four, two of our members dropped out and I assumed sole programming duties.

- ssd1306.py is modified by myself from an old Python 2 version I found on the internet. This is for the OLED display used on the vehicle.
- ultrasonic.py was provided by the uni and is for the Ultrasonic sensor that went ultimately unused in the final demonstration.
- motor.py was provided by the uni and modified by myself to provide a single ctrl_alloc() function used for all of the motor control.
- encoder.py was provided by the uni and is for the rotary encoder which measured the revolutions of each motor (1 per wheel). This was used to vary the motor speed automatically to ensure the vehicle drove straight.
- APDS9960LITE.py was provided by the uni and is for the Broadcom APDS9960 sensor module. This went unused in the final design.
- util.py was created by myself and includes a bunch of utility functions including the automatic motor speed compensation function and a variety of measurement/reporting functions used in development.
- main.py was created by myself and contains the majority of the vehicle code including pin config, tuning variables, state machine and all of the logic. It was designed to be modular and to make it easy to enable/disable different hardware modules using the variables at the start of the file.

My teammate and I received a mark of 75% for our demonstration.
