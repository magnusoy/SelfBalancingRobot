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

from communication import SerialCommunication
from joystick import Joystick
from pid import PID
from motor_controller import Motor
from robot import SelfBalancingRobot


motor = Motor()
mpu = SerialCommunication(port="COM3", baudrate=9600)
ps4Controller = Joystick()
pid = PID(input_=0.0, setPoint_=50.0, kp_=1.0, ki_=0.01, kd_=0.10, controllerDirection_="REVERSE")
robot = SelfBalancingRobot()


if __name__ == "__main__":
  while(mpu.isConnected() and ps4Controller.isConnected()):
    btnEvents = ps4Controller.readButtons()
    axesEvents = ps4Controller.readAxes()

    robot.onEvent(btnEvents)
    robot.onAction(axesEvents)

    pid.input_ = mpu.readInputStream()
    pid.compute()
    motor.left(pid.output)
    motor.right(pid.output)

    
