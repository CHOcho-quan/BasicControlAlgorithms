import numpy as np
import math

class FuzzyController:
    """
    This is a fuzzy controller in 1-Dimensional control

    """
    def __init__(self):
        """
        Initializing a 1 dimensional contrller
        self.controller : store the information inside each dimension including

        """
        self.controller = {}

    def setFuzzyLogic(self, T, max, min, control, func='triangle'):
        """
        Setting the fuzzy logic
        T : we cut the data into T parts
        max : maximum value of the n-th dimension
        min : minimum value of the n-th dimension
        control : T dimensional numpy array inferring the control signal
        func : choosing the fuzzy logic function containing only traingle now

        """
        self.controller['func'] = func
        self.controller['T'] = T
        self.controller['max'] = max
        self.controller['min'] = min
        self.controller['control'] = control

    def run(self, setpoint, sv):
        """
        Simulation of the fuzzy controller
        setpoint : input setpoint
        sv : sensor value

        """
        error = setpoint - sv
        output = None

        internal = float(self.controller['max'] - self.controller['min']) / float(self.controller['T'])
        if error <= self.controller['min']:
            output = self.controller['control'][0]
        elif error >= self.controller['max']:
            output = self.controller['control'][self.controller['T'] - 1]
        else:
            cnt = float(error - self.controller['min']) / float(internal)
            inf = int(cnt)
            upp = inf + 1
            inf_rate = (error - (self.controller['min'] + inf * internal)) / internal
            upp_rate = ((self.controller['min'] + upp * internal) - error) / internal
            output = inf_rate * self.controller['control'][inf] + upp_rate * self.controller['control'][upp]

        return output

if __name__ == '__main__':
    fc = FuzzyController()
    fc.setFuzzyLogic(3, 3, 0, [-1, 0, 1])
    fc.run(setpoint=1, sv=2)
