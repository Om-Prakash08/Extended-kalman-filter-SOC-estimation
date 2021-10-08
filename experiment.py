import numpy as np



class ExtendedKalmanFilter(object):

    def __init__(self, x, A, B, P, Q, R, Hx, HJacobian):

        self._x = x   # x = [[SoC], [RC voltage]]
        self._A = A   # state transition model
        self._B = B    # control-input model
        self._P = P     # state error covariance
        self._Q = Q      # process noise covariance matrix
        self._R = R     # measurement noise
        self._Hx = Hx
        self._HJacobian = HJacobian

        print("om",self.x, HJacobian(self.x),Hx(self.x))


    def update(self, z):

        P = self._P  # state error covariance
        R = self._R    # measurement noise
        x = self._x   # x = [[SoC], [RC voltage]]

        H = self._HJacobian(x)

        S = H * P * H.T + R
        K = P * H.T * S.I
        self._K = K

        hx =  self._Hx(x)
        y = np.subtract(z, hx)
        self._x = x + K * y

        KH = K * H
        I_KH = np.identity((KH).shape[1]) - KH
        self._P = I_KH * P * I_KH.T + K * R * K.T

    def predict(self, u=0):
        self._x = self._A * self._x + self._B * u
        self._P = self._A * self._P * self._A.T + self._Q

    @property
    def x(self):
        return self._x
