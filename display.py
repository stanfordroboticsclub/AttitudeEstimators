
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import numpy as np

import UDPComms
import time




class Quaternion:

    @classmethod
    def fromAxisAngle(cls, axis, angle):
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
        self.s = None

    def plot_quat(self,q):
        verts = [q.rotate(v) for v in self.verts]
        l = np.array(verts).T
        x = l[0].reshape((2,2))
        y = l[1].reshape((2,2))
        z = l[2].reshape((2,2))

        if self.s != None:
            self.s.remove()
        self.s = self.ax.plot_surface(x, y, z, linewidth=0, color='r',zorder=2)


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
        print(dt)
        self.last_time = time.time()
        gyro = [gx, gy, gz]
        angle = np.linalg.norm(gyro) * dt
        return Quaternion.fromAxisAngle( gyro, angle )

    def get_acel_vect(self):
        try:
            ax,ay,az, gx, gy, gz = self.sub.get()
        except UDPComms.timeout:
            return None

        return np.array([ax, ay, az])


class Integrator:
    def __init__(self):
        self.q = Quaternion.fromAxisAngle( [0,0,1], 0 )

    def update_gyro(self, gyro_quat):
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

if __name__ == "__main__":
    imu = IMU()
    display = Display()
    filt = Integrator()

    while 1:
        filt.update_gyro( imu.get_gyro_quat() )
        filt.update_acel( imu.get_acel_vect() )
        display.plot_quat( filt.quat() )
        plt.pause(0.01)

