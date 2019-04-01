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

from states import InitState

class SelfBalancingRobot(object):
    """ 
    A state machine for different behaviours.
    """

    def __init__(self):
        """ 
        Initialize the components.
        """

        # Start with a default state.
        self.state = InitState()

    def onEvent(self, event):
        """
        Incoming events are delegated to the given states
        which then handle the event. The result is then 
        assigned as the new state.
        """

        # The next state will be the result of the on_event function.
        self.state = self.state.onEvent(event)
    
    def onAction(self, actionEvent):
        """
        Incoming action events are delegated to the given states
        which then handle the action.
        """

        self.action = self.state.onAction(actionEvent)
