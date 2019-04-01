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

# Importing libraries
import pygame
import numpy as np


class Joystick(object):
    """
    Game controller handler.
    """

    pygame.init()
    pygame.joystick.init()
    pygame.display.get_init()

    def __init__(self):
        """
        Constructor used to inizilize a pygame and joystick object.
        Require a controller to be connected through Serial or Bluetooth.
        """

        self.clock = pygame.time.Clock()

        try:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.numOfBtns = self.joystick.get_numbuttons()
            self.numOfAxes = self.joystick.get_numaxes()
        except pygame.error:
            self.joystick = None
            print("Could not connect to joystick")

    def __str__(self):
        """
        docstring
        """

        return "{} is connected".format(self.joystick.get_name())

    def isConnected(self):
        """
        Checks if controller is connected.
        @return False if not connected
                else returns True
        """

        result = False
        if self.joystick is not None:
            result = True
        return result

    def readAxes(self):
        """
        Reads the analog axes from the controller.
        @return an array of read float values
        """

        axes = np.zeros(self.numOfAxes)
        for event in pygame.event.get():
            for index in range(self.numOfAxes):
                axes[index] = self.joystick.get_axis(index)
        return axes

    def readButtons(self):
        """
        Reads the digital button presses from the controller.
        @return an array storing the states of the buttons
        """

        buttons = np.zeros(self.numOfBtns)
        for event in pygame.event.get():
            for index in range(self.numOfBtns):
                buttons[index] = self.joystick.get_button(index)
        return buttons

    def updateRate(self, rate):
        """
        Adjust the update rate on the readings.
        """

        self.clock.tick(rate)

    def disconnect(self):
        """
        Disconnects the controller.
        Destroying the game object.
        """

        pygame.quit()


# Example of usages
if __name__ == "__main__":
    ps4Controller = Joystick()

    while(ps4Controller.isConnected()):
        ps4Controller.updateRate(20)
        print(ps4Controller.readButtons())
