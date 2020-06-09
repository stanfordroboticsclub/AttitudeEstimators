from quaternion import Quaternion
import numpy as np

class ComplementaryFilter:
    def __init__(self):
        self.q = Quaternion.Identity()

    def update_gyro(self, w, dt):
        gyro_quat = Quaternion.fromGyro(w, dt)
        self.q =  self.q @ gyro_quat

    def update_acel(self, accel_vect):
        alpha = 0.01

        if accel_vect is None:
            return
        sim = self.q.T.rotate( [0,0,-1] )

        dot_product =  np.sum(sim * accel_vect) / np.linalg.norm(sim) / np.linalg.norm(accel_vect)
        angle = np.arccos(dot_product)

        # to modify the sim we go from the real to simulated gravity vector
        axis = np.cross(accel_vect, sim)

        offset = Quaternion.fromAxisAngle(axis, alpha*angle)
        self.q =  self.q @ offset


    def quat(self):
        return self.q

class MEKF:
    def __init__(self):
        self.q = Quaternion.Identity()

        self.bias = np.array([0.0,0.0,0.0])

        self.sigma = np.diag([0.1, 0.1, 0.1, 0.5, 0.5, 0.5])

        # self.Q = 0.02 * np.eye(6) * 0.05
        self.Q = np.diag( [0.02 * 0.05] * 3 + [1e-12]*3 )
        self.R = 10  * np.eye(3)

    def update_gyro(self, w, dt):

        gyro_quat = Quaternion.fromGyro( w - self.bias, dt)

        # A = [[ Matrix, - 0.5 * dt * I], [0, I]]
        A = np.block([[gyro_quat.get_matrix(), -0.5 * np.eye(3) * dt],
                      [ np.zeros((3,3))     , np.eye(3)]])


        # sketchy might need to change to doing it component wise
        self.q =  self.q @ gyro_quat
        self.sigma = A @ self.sigma @ A.T + self.Q

        self.q = self.q.normalize()


    def update_acel(self, accel_vect):
        if accel_vect is None:
            return

        # unsure why it works best with gravity vectors of length 1
        # sim = self.q.T.rotate( [0,0,-9.8] )
        sim = self.q.T.rotate( [0,0,-1] )
        # sim = self.q.T.rotate( [0,0,-1] ) * np.linalg.norm(accel_vect)
        accel_vect = accel_vect/np.linalg.norm(accel_vect)
        # accel_vect = accel_vect/9.8

        print("sim",sim)
        print("accel", accel_vect)

        # has nothing to do with quaternion
        # just using a convenience function
        C = 2*Quaternion.skew_matrix(sim)
        C = np.block( [[ C, np.zeros((3,3)) ]])

        print("start")
        # print("sigma", self.sigma)
        # print("C", C)
        K = self.sigma @ C.T @ np.linalg.inv(C @ self.sigma @ C.T + self.R)
        # print("K",K)

        correction = K@(accel_vect - sim)
        nudge = correction[:3]
        bias_fix = correction[3:]

        print(bias_fix.shape)

        self.bias += bias_fix

        # print("nudge",nudge)
        print("covars", np.diag(self.sigma))
        print("bias", self.bias)

        self.q = self.q @ Quaternion.fromNudge(nudge)
        self.sigma = self.sigma - K @ C @ self.sigma


    def quat(self):
        return self.q
