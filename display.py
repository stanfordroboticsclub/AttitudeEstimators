from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import numpy as np

import UDPComms
import time




class Quaternion:
    
    def __str__(self):
        return str(self.q)

    @classmethod
    def Identity(cls):
        return Quaternion( [1, 0, 0, 0] )

    @classmethod
    def fromGyro(cls, w, dt):
        angle = np.linalg.norm(w) * dt
        return Quaternion.fromAxisAngle( w, angle )

    @classmethod
    def fromNudge(cls, nudge):
        w = np.sqrt(1 - np.linalg.norm(nudge)**2)
        return Quaternion( [w, nudge[0], nudge[1], nudge[2]] )

    @classmethod
    def fromAxisAngle(cls, axis, angle):
        if(angle == 0):
            return Quaternion.Identity()
        axis = axis/np.linalg.norm(axis)
        c = np.cos(angle/2)
        s = np.sin(angle/2)
        return Quaternion( [c, s * axis[0], s * axis[1], s * axis[2]] )

    def __init__(self, array):
        self.q = np.array(array)

    def __matmul__(self, other):
        w0, x0, y0, z0 = other.q
        w1, x1, y1, z1 = self.q
        return Quaternion([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                         x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                         -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                         x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0])

    @property
    def T(self):
        w, x, y, z = self.q
        return Quaternion([w, -x, -y, -z])

    def get_matrix(self):
        w = self.q[0]
        v = self.q[1:]

        return (w**2 - np.linalg.norm(v)**2) * np.eye(3) \
                    + 2 * np.outer(v, v) \
                    + 2 * w * self.skew_matrix(v)

    @staticmethod
    def skew_matrix(v):
        # also the cross product matrix
        x, y, z = v
        return np.array ([[ 0,-z, y],
                          [ z, 0,-x],
                          [-y, z, 0]])

    def normalize(self):
        return Quaternion( self.q / np.linalg.norm(self.q) )

    def rotate(self, vector):
        assert len(vector) == 3
        tmp = Quaternion([0, vector[0], vector[1], vector[2]])
        out = self @ tmp @ self.T
        return out.q[1:]


class Display:

    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.gca(projection='3d')

        X = np.array([[-1 ,1],[-1, 1]])
        Y = np.array([[ 1 ,1],[-1,-1]])
        Z = np.array([[-1,-1],[-1,-1]])

        scale = 3
        self.ground = self.ax.plot_surface(scale* X, scale*Y, scale*Z,
                                           color='g',zorder=1)
        bounds = scale + 0.01
        self.ax.set_zlim(-bounds, bounds)

        self.verts = [[-1, 2, 0],
                      [-1,-2, 0],
                      [ 1, 2, 0],
                      [ 1,-2, 0]]
        self.arrow = [0,0,1]
        self.s = None
        self.a = None


    def plot_quat(self,q):
        verts = [q.rotate(v) for v in self.verts]
        l = np.array(verts).T
        x = l[0].reshape((2,2))
        y = l[1].reshape((2,2))
        z = l[2].reshape((2,2))

        if self.s != None:
            self.s.remove()
        if self.a != None:
            self.a.remove()

        self.s = self.ax.plot_surface(x, y, z,
                    linewidth=0, color='r',zorder=2, alpha=0.8)
        arrow = q.rotate(self.arrow)
        self.a = self.ax.quiver(
            0, 0, 0,
            *arrow,
            color = 'blue', alpha = .8, lw = 3,zorder = 3)


class IMU:

    def __init__(self):
        self.sub = UDPComms.Subscriber(8007)
        self.last_time = time.time()

    def get_gyro_quat(self):
        try:
            ax,ay,az, gx, gy, gz = self.sub.get()
        except UDPComms.timeout:
            return Quaternion.fromAxisAngle( [0,0,1], 0)
        dt = time.time() - self.last_time
        print("dt: ", dt)
        self.last_time = time.time()
        return Quaternion.fromGyro( [gx,gy,gz], dt)

    def get_gyro_vect(self):
        try:
            ax,ay,az, gx, gy, gz = self.sub.get()
        except UDPComms.timeout:
            return [0,0,0], 0
        dt = time.time() - self.last_time
        print("dt: ", dt)
        self.last_time = time.time()
        return [gx, gy, gz], dt

    def get_acel_vect(self):
        try:
            ax,ay,az, gx, gy, gz = self.sub.get()
        except UDPComms.timeout:
            return None

        return np.array([ax, ay, az])


class ComplementaryFilter:
    def __init__(self, alpha=0.1):
        self.q = Quaternion.Identity()
        self.alpha = alpha

    def update_gyro(self, w, dt):
        gyro_quat = Quaternion.fromGyro(w, dt)
        self.q =  self.q @ gyro_quat

    def update_acel(self, accel_vect):
        alpha = self.alpha

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
    def __init__(self, verbose=0, Q_gyro=1e-6, Q_bias=1e-12, R=100):
        self.q = Quaternion.Identity()

        self.bias = np.array([0.0,0.0,0.0])

        self.sigma = np.diag([0.1, 0.1, 0.1, 0.5, 0.5, 0.5])

        # self.Q = 0.02 * np.eye(6) * 0.05
        self.Q = np.diag( [Q_gyro] * 3 + [1e-9]*3 )
        self.R = R * np.eye(3)

        self.verbose = verbose

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
        
        if self.verbose > 0:
            print("sim",sim)
            print("accel", accel_vect)

        # has nothing to do with quaternion
        # just using a convenience function
        C = 2*Quaternion.skew_matrix(sim)
        C = np.block( [[ C, np.zeros((3,3)) ]])

        #print("start")
        # print("sigma", self.sigma)
        # print("C", C)
        K = self.sigma @ C.T @ np.linalg.inv(C @ self.sigma @ C.T + self.R)
        # print("K",K)

        correction = K@(accel_vect - sim)
        nudge = correction[:3]
        bias_fix = correction[3:]

        self.bias += bias_fix

        # print("nudge",nudge)
        if self.verbose > 0:
            print("covars", np.diag(self.sigma))
            print("bias", self.bias)

        self.q = self.q @ Quaternion.fromNudge(nudge)
        self.sigma = self.sigma - K @ C @ self.sigma


    def quat(self):
        return self.q

if __name__ == "__main__":
    imu = IMU()
    display = Display()
    # filt = ComplementaryFilter()
    filt = MEKF()

    t = 0

    while 1:
        filt.update_gyro( *imu.get_gyro_vect() )
        filt.update_acel( imu.get_acel_vect() )

        if t % 5 == 0:
            display.plot_quat( filt.quat() )
        
        plt.pause(0.01)
        t += 1

