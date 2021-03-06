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
import time


class PID(object):
    """
    A fully functional PID - controller.
    """

    def __init__(self, input_, setPoint_=50-0, kp_=1.0, ki_=0.0, kd_=0.0, controllerDirection_="FORWARD"):
        """
        Constructor used to initialize necessary parameters to have a working controller.
        @input_ : feedback from the closed loop
        @setPoint_ : desired value
        @kp_ : proportional term
        @ki_ : integral term
        @kd_ : derivative term
        @controllerDirection_ : FORWARD or REVERSE
        """
        
        self.input = input_
        self.setPoint = setPoint_
        self.inAuto = False
        self.sampleTime = 100  # In millis
        self.controllerDirection = "FORWARD"
        self.setControllerDirection(controllerDirection_)
        self.setTunings(kp_, ki_, kd_)
        self.lastTime = self.millis() - self.sampleTime
        self.output = 0
        self.outputSum = 0
        self.lastInput = 0
        self.outMin, self.outMax = 0, 0
    
    def millis(self):
        """
        Returns the current time in milliseconds.
        """

        return int(round(time.time() * 1000))
    
    def setTunings(self, kp_, ki_, kd_):
        """
        Set the PID - controller terms.
        @kp_ : proportional term
        @ki_ : integral term
        @kd_ : derivative term
        """

        if (kp_ < 0 or ki_ < 0 or kd_ < 0): return

        sampleTimeInSec = self.sampleTime / 1000 
        self.kp = kp_
        self.ki = ki_ * sampleTimeInSec
        self.kd = kd_ * sampleTimeInSec

        if(self.controllerDirection == "REVERSE"):
            self.kp = (0 - kp_)
            self.ki = (0 - ki_)
            self.kd = (0 - kd_)
    
    def compute(self):
        """
        Calculates the output by the given terms.
        @return False if time since last change is less than sample time.
                else returns True when output is updated.
        """

        computed = False
        if(not self.inAuto): return False
        now = self.millis()
        timeChange = now - self.lastTime
        if(timeChange >= self.sampleTime):

            input_ = self.input
            error = self.setPoint - input_
            dInput = input_ - self.lastInput
            self.outputSum += self.ki * error

            if(self.outputSum > self.outMax): self.outputSum = self.outMax
            elif(self.outputSum < self.outMin): self.outputSum = self.outMin
            
            output = self.kp * error
            output += self.outputSum - (self.kd * dInput)

            if(output > self.outMax): output = self.outMax
            elif(output < self.outMin): output = self.outMin
            self.output = output

            self.lastInput = input_
            self.lastTime = now

            computed = True

        return computed
    
    def setMode(self, mode):
        """
        Set the PID - controller to Automatic.
        @mode : AUTOMATIC or MANUAL
        """

        newAuto = False
        if (mode == "AUTOMATIC"): newAuto = True

        if(newAuto and not self.inAuto):
            self.initialize()
        self.inAuto = newAuto
    
    def setSampleTime(self, newSampleTime):
        """
        Set the time when the PID output should update.
        @newSampleTime : time in milliseconds
        """

        if (newSampleTime > 0):
            ratio = newSampleTime / self.sampleTime
            self.ki *= ratio
            self.kd /= ratio
            self.sampleTime = newSampleTime
    
    def setOutputLimits(self, min_, max_):
        """
        Constrain the PID output to the given min and max.
        @min_ : lowest value
        @max_ : highest value
        """

        if (min_ > max_): return
        self.outMin = min_
        self.outMax = max_

        if(self.inAuto):
            if(self.output > self.outMax): self.output = self.outMax
            elif(self.output < self.outMin): self.outputSum = self.outMin
            
            if(self.outputSum > self.outMax): self.outputSum = self.outMax
            elif(self.outputSum < self.outMin): self.outputSum = self.outMin


    def initialize(self):
        """
        Initializes the PID when moving from MANUAL to AUTOMATIC.
        """

        self.outputSum = self.output
        self.lastInput = self.input
        if(self.outputSum > self.outMax): self.outputSum = self.outMax
        elif(self.outputSum < self.outMin): self.outputSum = self.outMin

    
    def setControllerDirection(self, newDirection):
        """
        Set the PID - controllers direction.
        @newDirection : FORWARD or REVERSE
        """

        if(self.inAuto and (newDirection != self.controllerDirection)): 
            self.kp = (0 - self.kp)
            self.ki = (0 - self.ki)
            self.kd = (0 - self.kd)
        
        self.controllerDirection = newDirection


# Example of usage
if __name__ == "__main__":
    pid = PID(input_=20.0, setPoint_=50.0, kp_=1.0, ki_=0.01, kd_=0.10, controllerDirection_="REVERSE")
    pid.setOutputLimits(0, 100)
    pid.setMode("AUTOMATIC")
    i = 0
    while(i <= 100000):
        pid.compute()
        i += 1