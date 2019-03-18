import numpy as np
import matplotlib.pyplot as plt

class KalmanFilter:
    """
    A simple Kalman filter implementation

    """

    def __init__(self, A, B, C, dim_x, dim_z, dim_u = 0):
        """
        Initialization of system: x_k = Ax_l + Bu
        Q : noise matrix with covarience
        P & P_l : covarience now and last time
        R : Kalman Gain error
        Kg : Kalman Gain

        """
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.dim_u = dim_u

        self.x_k = np.zeros((dim_x, 1))
        self.x_l = np.zeros((dim_x, 1))
        self.Q = np.eye(dim_x)

        self.P = np.eye(dim_x)
        self.P_l = np.eye(dim_x)
        self.Kg = np.zeros((dim_x,dim_z))
        self.R = np.eye(dim_z)

        self.A = A
        self.B = B
        self.C = C

    def predict(self, u, y):
        """
        With measurement y & input u
        we predict the next state base on priori and measurement

        """
        # Prediction part
        self.x_k = (self.A).dot(self.x_k) + (self.B).dot(u)
        self.P = self.A.dot(self.P_l).dot(self.A.T) + self.Q

        # Updating part
        # Calculating the Kalman Gain
        self.Kg = self.P.dot(self.C.T).dot(np.linalg.inv(self.C.dot(self.P).dot(self.C.T) + self.R))
        self.x_k = self.x_k + self.Kg.dot(y - self.C.dot(self.x_k))
        # Calculating P post
        self.P = (np.eye(self.dim_x) - self.Kg.dot(self.C))

        return self.x_k

    def getOutput(self):
        return self.x_k

if __name__ == '__main__':
    kf = KalmanFilter(np.array([1]), np.array([1]), np.array([1]), 1, 1, 1)
    for i in range(10):
        kf.predict(np.array([3]), np.array([4]))
        print("Prediction:", kf.getOutput())
