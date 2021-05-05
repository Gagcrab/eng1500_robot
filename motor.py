class Motor(object):
    """ A ``motor`` object is used to control a DC motor using the L298N dual H-bridge.
    The Motor class has methods to change the direction of the motor by setting
    the IN1/IN2 pins:
    - ``Motor.set_forwards()``
    - ``Motor.set_backwards()``
     and a method to change the duty cycle (0-100%) of the PWM output:
    - ``Motor.duty(pwm)``
    """

    def __init__(self, side, in1_pin, in2_pin, en_pin):
        """
        :rtype: object
        :type side: str
        :type in1_pin: str
        :type in2_pin: str
        :type en_pin: str
        The arguments are:
        - ``side``  should be the strings: "left" or "right". Used to specify motor.
        - ``in1_pin`` should be a valid Pin name string.
        - ``in2_pin`` should be a valid Pin name string
        - ``en_pin`` should be the strings: "D4" or "D5"
        Usage Model::
        # Initialise motor instance
        left_motor = Motor("left", "D6", "D7", "D4")
        # Set motor direction to forwards, and duty cycle to 70%
        left_motor.set_forwards()
        left_motor.duty(70)
        Note: This module assumes your en_pin is attached to either D4 or D5.
        To use another pin, you must choose the correct timer & channel
        appropriate to that pin. For a list of pin/timer/channel pairings,
        please see the pin-out diagram for the Nucleo L476RG:
        https://os.mbed.com/platforms/ST-Nucleo-L476RG/#arduino-compatible-headers
        E.g. The Arduino pin D2 is labelled `PWM1/3' indicating Timer 1, Channel 3.
        Modifications must be made to this library as appropriate, and this is
        left as an open problem for the student :) Have fun!
        """
        from pyb import Timer
        from machine import Pin
        # Declare a variable to keep track of which motor is left/right
        self.side = side
        print("Initialising", self.side, "motor...")
        # Declare GPIO pins for direction IN1 and IN2 pins
        self.IN1 = Pin(in1_pin, Pin.OUT)
        self.IN2 = Pin(in2_pin, Pin.OUT)
        # Declare GPIO pins for PWM EN pin
        self.EN = Pin(en_pin)
        # Set timer number and channel depending on pin chosen as input
        if en_pin == "D4":
            _tim = 3
            _channel = 2
        elif en_pin == "D5":
            _tim = 3
            _channel = 1
        # Declare a Timer instance for PWM output on EN pin and configure the channel
        self.tim = Timer(_tim, freq=20000)  # Frequency at 20kHz to be outside human hearing
        self.ch = self.tim.channel(_channel, Timer.PWM, pin=self.EN)
        print("Motor", self.side, "initialised!")

    def duty(self, pwm):
        self.ch.pulse_width_percent(pwm)

    def set_forwards(self):
        if self.side == "left":
            self.IN1.on()
            self.IN2.off()
        else:  # only other motor is the right motor
            self.IN1.off()
            self.IN2.on()

    def set_backwards(self):
        if self.side == "left":
            self.IN1.off()
            self.IN2.on()
        else:
            self.IN1.on()
            self.IN2.off()

    def ctrl_alloc(self, pwmvec):
        """
        Controls speed and direction of a Motor object. This should be
        used in place of the duty(), set_forwards() and set_backwards()
        functions.
        :param pwmvec: Percentage of full power to be sent as a PWM
        voltage. Sign determines direction and absolute value determines
        power.
        """
        if pwmvec >= 0:  # Forward
            self.set_forwards()
        else:            # Backward
            self.set_backwards()

        pwm = abs(pwmvec)
        if 0 <= pwm <= 100:
            self.duty(pwm)
