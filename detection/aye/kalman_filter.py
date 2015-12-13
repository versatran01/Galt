from __future__ import print_function, division, absolute_import
import numpy as np
import numpy.linalg as la


class KalmanFilter(object):
    def __init__(self, x):
        self.x = x
        # State covariance
        p = 5.0
        self.P = np.diag([p, p])
        # Process covariance
        q = 3.0
        self.Q = np.diag([q, q])
        # Measurement covariance
        r = 1.0
        self.R = np.diag([r, r])

        self.xs = []

    def predict(self, u):
        # F is I2, B is I2
        # x <- F * x + B * u
        self.x += u
        # P <- F * P * F^T + Q
        self.P += self.Q

    def correct(self, z):
        # H is I2
        # y = z - H * z
        y = z - self.x
        # S = H * P * H^T + R
        S = self.P + self.R
        # K = P * H^T * S^-1
        K = self.P * la.inv(S)
        # x <- x + K * y
        self.x += np.dot(K, y)
        # P <- (I - K * H) * P
        self.P = np.dot(np.identity(2) - K, self.P)

        # Save state
        self.xs.append(self.x)
