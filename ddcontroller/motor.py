#!/usr/bin/env python3

'''
This file is part of the ddcontroller library (https://github.com/ansarid/ddcontroller).
Copyright (C) 2022  Daniyal Ansari

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''

import RPi.GPIO as GPIO
GPIO.setwarnings(False)

class Motor:
    """Class for controlling a motor using the Raspberry Pi's GPIO pins.

    Args:
        pins (list): A list of the GPIO pins that will be used to control the motor.
        pwm_frequency (int): The frequency of the pulse width modulation (PWM) signal
            that will be used to control the motor.
        initial_duty (int, optional): The initial duty cycle of the PWM signal. Defaults to 0.
        decay_mode (str, optional): The decay mode of the motor. Can be either "FAST" or
            "SLOW". Defaults to "FAST".
        invert (bool, optional): A boolean value indicating whether the motor's direction
            should be reversed. Defaults to False.
        rpm (int, optional): The speed of the motor in rotations per minute. Defaults to 200.
        control_type (str, optional): Set to DUAL_PWM for H-bridge, and PWM_DIR for using a
            one PWM signal and a direction control signal. Defaults to DUAL_PWM. If set to
            PWM_DIR, then pins[0] will be a PWM and pins[1] will be the direction control and
            the 'invert' argument will only affect the polarity of the direction control signal.
        min_duty (int, optional): The minimum duty cycle in percent for the PWM to create
            motion in the motor. Defaults to 0.
        max_duty (int, optional): The maximum duty cycle in percent. Defaults to 100.
    """

    def __init__(self, pins, pwm_frequency, initial_duty=0, decay_mode='FAST', invert=False, rpm=200, control_type='DUAL_PWM', min_duty=0, max_duty=100):
        self.pins = pins
        self._pins = []

        # Initial Duty %
        self.duty = initial_duty

        # PWM frequency (Hz)
        self.pwm_frequency = pwm_frequency

        # Decay Mode (FAST/SLOW)
        self.decay_mode = decay_mode

        # Reverse motor direction
        self.invert = invert

        # Motor RPM
        self.rpm = rpm

        # Control mode
        self.control_type = control_type

        # Min motor duty
        self.min_duty = min_duty/100

        # Max motor Duty
        self.max_duty = max_duty/100

        if GPIO.getmode() is None:
            GPIO.setmode(GPIO.BOARD)
        else:
            pass

        if self.control_type == "DUAL_PWM":
            for pin in self.pins:  # Set motor pins as outputs
                GPIO.setup(pin, GPIO.OUT)
                self._pins.append(GPIO.PWM(pin, self.pwm_frequency))
        else:  # Otherwise, assume control_type to be "PWM_DIR"
            # setup first pin as PWM
            GPIO.setup(self.pins[0], GPIO.OUT)
            self._pins.append(GPIO.PWM(self.pins[0], self.pwm_frequency))
            # and second pin as a GPIO output
            GPIO.setup(self.pins[1], GPIO.OUT)

        for pin in self._pins:
            pin.start(self.duty)

    def set_pwm_frequency(self, frequency):
        """Sets the frequency of the PWM signal.

        Args:
            frequency (int): The new frequency of the PWM signal.
        """
        self.pwm_frequency = frequency
        for pin in self._pins:
            pin.ChangeFrequency(self.pwm_frequency)

    def set_duty(self, duty):
        """Sets the duty cycle of the PWM signal.

        Args:
            duty (float): The new duty cycle of the PWM signal, from -1 to 1.

            If min_duty and max_duty are set, then the PWM signal output
            is scaled to this range.
            
            For example, if  min_duty is 40 (40%) and the max_duty is 95 (95%), then:
            duty = .01, PWM duty will be 40% (minimum)
            duty = 1, PWM duty will be 95% (maximum)
            duty = 0, PWM duty will be 0 (off)
        """
        self.duty = round(  # Make sure duty is between -1 and 1
            sorted((-1, float(duty), 1))[1], 2
        )

        # Get the sign (1.0 or -1.0) of the duty cycle (number between -1.0 to +1.0)
        sign = -1.0 if self.duty < 0 else 1.0

        # Map the duty cycle to the min and max duty range
        duty = 100 * sign * (abs(self.duty) * (self.max_duty-self.min_duty) + self.min_duty) 

        if self.decay_mode == 'SLOW':

            if duty == 0:
                for pin in self._pins:
                    pin.ChangeDutyCycle(100)

            elif duty > 0:
                if self.control_type == 'DUAL_PWM':
                    self._pins[0].ChangeDutyCycle(100)
                    self._pins[1].ChangeDutyCycle(100-duty)
                else:
                    self._pins[0].ChangeDutyCycle(100-duty)
                    GPIO.output(self.pins[1], 1 if self.invert == 'False' else 0)

            elif duty < 0:
                if self.control_type == 'DUAL_PWM':
                    self._pins[0].ChangeDutyCycle(100-abs(duty))
                    self._pins[1].ChangeDutyCycle(100)
                else:
                    self._pins[0].ChangeDutyCycle(100-abs(duty))
                    GPIO.output(self.pins[1], 0 if self.invert == 'False' else 1)

        elif self.decay_mode == 'FAST':

            if duty == 0:
                for pin in self._pins:
                    pin.ChangeDutyCycle(0)

            elif duty > 0:
                if self.control_type == 'DUAL_PWM':
                    self._pins[0].ChangeDutyCycle(0)
                    self._pins[1].ChangeDutyCycle(duty)
                else:
                    self._pins[0].ChangeDutyCycle(duty)
                    GPIO.output(self.pins[1], 1 if self.invert == False else 0)                   

            elif duty < 0:
                if self.control_type == 'DUAL_PWM':
                    self._pins[0].ChangeDutyCycle(abs(duty))
                    self._pins[1].ChangeDutyCycle(0)
                else:
                    self._pins[0].ChangeDutyCycle(abs(duty))
                    GPIO.output(self.pins[1], 0 if self.invert == False else 1)                

        else:
            print('Invalid Decay Mode!')

    def stop(self):
        """Stops the motor by setting the duty cycle of the PWM signal to 0."""

        for pin in self._pins:
            pin.stop()

        GPIO.cleanup(self.pins)