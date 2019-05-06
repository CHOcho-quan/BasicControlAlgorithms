import numpy as np
import math

class LP:
    """
    A very simple Linear Programming class only for the standard form of LP
    standard form requests:
    1.Maximize the object functino
    2.Constraints are all leq
    3.Constraints & variables are non-negative

    """
    def __init__(self, objective, constraints):
        """
        objective : 1 x (n_variables + 1) array corresponding the objective function
        constraints : n_constraints x (n_variables + 1) array corresponding the constraints

        """
        self.num_basic_variables = constraints.shape[0]
        self.num_variables = objective.shape[0] - 1
        self.num_constraints = constraints.shape[0]

        self.tableau = np.zeros(shape=(self.num_basic_variables + 1, 3 + self.num_variables + self.num_constraints))
        for i in range(self.tableau.shape[1]):
            if (i == 0):
                self.tableau[0, i] = 1
                continue
            elif (i == self.tableau.shape[1] - 2):
                self.tableau[0, i] = 0
                continue
            elif (i == self.tableau.shape[1] - 1):
                self.tableau[0, i] = math.inf
                continue

            if (i <= self.num_variables):
                self.tableau[0, i] = -objective[i-1]
            elif (i > self.num_variables):
                self.tableau[0, i] = 0

        for i in range(1, self.tableau.shape[0]):
            for j in range(self.tableau.shape[1] - 2 - self.num_constraints):
                if (j == 0):
                    self.tableau[i, 0] = 0
                    continue

                self.tableau[i, j] = constraints[i-1, j-1]
                self.tableau[i, self.num_variables + i] = 1

        self.tableau[:, -1] = math.inf
        self.tableau[1:, -2] = constraints[:, -1]
        print("Constructed tableau: \n", self.tableau)

    def _checkOpt(self):
        """
        Check if reach optimized point

        """
        return np.count_nonzero(self.tableau[0, :] < 0)

    def _gaussianEliminate(self, eV, lV):
        """
        Do Gaussain Elimination to reach the optimization goal

        """
        waitList = np.nonzero(self.tableau[:, eV])[0]
        for i in waitList:
            if i != lV:
                self.tableau[i, :-1] -= self.tableau[i, eV] * self.tableau[lV, :-1]
        self.tableau[:, -1] = math.inf

    def optimize(self):
        """
        Optimize the standard LP problem

        """
        while (self._checkOpt()):
            enteringV = np.where(self.tableau[0, :]==np.min(self.tableau[0, :]))[0][0]
            self.tableau[1:, -1] = self.tableau[1:, -2] / (self.tableau[1:, enteringV] + 0.0001)
            leavingV = np.where(self.tableau[:, -1]==np.min(self.tableau[:, -1]))[0][0]
            # print(enteringV, leavingV)
            self._gaussianEliminate(enteringV, leavingV)
            print(self.tableau)
        print("Reach Optimized Point!", self.tableau[0, -2])


if __name__ == "__main__":
    lp = LP(np.array([15, 10, 0]), np.array([[1, 0, 2], [0, 1, 3], [1, 1, 4]]))
    lp.optimize()
