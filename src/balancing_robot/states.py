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

from state_interface import State

# Start of our states
class InitState(State):
    """
    The state which indicates that there are limited device capabilities.
    """

    def onEvent(self, event):
        print("InitState")
        if int(event[0]) == 1:
            return StandbyState()

        return self
    
    def onAction(self, actionEvent):
        """
        docstring
        """
        
        return self


class StandbyState(State):
    """
    The state which indicates that there are no limitations on device
    capabilities.
    """

    def onEvent(self, event):
        print("StandbyState")
        if int(event[8]) == 1:
            return RunningState()

        return self
    
    def onAction(self, actionEvent):
        """
        docstring
        """
        

        return self


class RunningState(State):
    """
    The state which indicates that there are no limitations on device
    capabilities.
    """

    def onEvent(self, event):
        if event == 'SQUARE':
            return StandbyState()

        return self
    
    def onAction(self, actionEvent):
        """
        docstring
        """

        return self