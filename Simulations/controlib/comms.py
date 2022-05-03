"""

Author: Francisco Branco
Created: 15/03/2021

"""

import numpy as np
from math import pi
import control as ct
import utils


class CommunicationBlock(ct.NonlinearIOSystem):
    """
    Communication type determines what kind of output is calculated:
        - 0 for continuous communication
    """
    def __init__(self, communication_type, delay=0):
        self.gamma = []
        self.comms = []
        self.communication_type = communication_type
        self.start_time = 0
        self.delay = delay
        super().__init__(updfcn=None, outfcn=self.comms_output, outputs=("gamma", "comms"))

    def simple_delay_output(self, t):
        if t - self.start_time < self.delay:
            
            self.start_time = t
        return 

    def comms_output(self, t, x, u, params={}):
        if self.communication_type == 0:
            self.simple_delay_output(t)