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


# ps4Controller = Joystick()
arduino = SerialCommunication(port="COM10", baudrate=115200)


if __name__ == "__main__":
  while(arduino.isConnected()):
    # ps4Controller.updateRate(20)
    # buttonStates = ps4Controller.readButtons()
    #buttonValues = [buttonStates[0],
    #                buttonStates[1],
    #                buttonStates[2],
    #                buttonStates[3],
    #                buttonStates[4]]

    buttonValues = [1, 1, 0, 0, 0]
    
    data = ','.join(map(str, buttonValues))
    arduino.sendOutputStream(data)
    print(arduino.readInputStream())
