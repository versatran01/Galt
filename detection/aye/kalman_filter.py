from __future__ import print_function, division, absolute_import
import numpy as np


class KalmanFilter(object):
    def __init__(self):
        self.X = np.empty([4, 1])
        self.P = np.empty([4, 4])

    def init(self, y):
        self.X = y
        self.P = np.identity(4)

    def predict(self, dt):
        A = np.matrix('0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0')
        F = np.identity(4) + dt * A

        # predict next state and covariance
        self.X = F.dot(self.X)
        self.P = F.dot(self.P).dot(np.transpose(F)) 

    def update(self, z):
        H = np.matrix('1 0 0 0; 0 1 0 0')
        R = np.identity(2)  # TODO: input some covariance for measurements

        # calculate residual and covariance of residual
        y = z - H.dot(self.X)  # residual
        S = H.dot(self.P).dot(np.transpose(H)) + R

        # calculate Kalman Gains
        K = self.P.dot(np.transpose(H)).dot(np.linalg.inv(S))

        # update state and covariance
        self.X = self.X + K.dot(y)
        self.P = (np.identity(4) - K.dot(H)).dot(self.P)
