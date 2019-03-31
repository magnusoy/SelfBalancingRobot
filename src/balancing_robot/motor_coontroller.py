# #!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
  MIT License

  Copyright (c) 2019 magnusoy

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
"""

# Importing library
from adafruit_motorkit import MotorKit


class Motor(object):
    """
    Makes use of the Raspberry Pi Motor Hat package.
    """

    def __init__(self):
        """
        Initilizes the Motor Hat.
        """

        self.kit = MotorKit()

    def left(self, speed):
        """
        Adjust the speed on the left motor.
        @speed : between 0.0 and 1.0
        """

        self.kit.motor1.throttle = speed

    def right(self, speed):
        """
        Adjust the speed on the right motor.
        @speed : between 0.0 and 1.0
        """

        self.kit.motor2.throttle = speed
    
    def spin(self):
        """
        Let the motor coast and then spin freely.
        """

        self.kit.motor1.throttle = None
        self.kit.motor2.throttle = None


# Example of usage
if __name__ == "__main__":
    i = 0
    while(i < 10000):
        motor = Motor()
        motor.left(1.0)
        motor.right(1.0)
        i += 1