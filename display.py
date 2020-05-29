
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import numpy as np




class Quaternion:

    @classmethod
    def fromAxisAngle(cls, axis, angle):
        axis = axis/np.sum(axis)
        c = np.cos(angle/2)
        s = np.sin(angle/2)
        return Quaternion( [c, s * axis[0], s * axis[1], s * axis[2]] )

    def __init__(self, array):
        self.q = np.array(array)

    def __matmul__(self, other):
        w0, x0, y0, z0 = self.q
        w1, x1, y1, z1 = other.q
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


fig = plt.figure()
ax = fig.gca(projection='3d')

plt.ion()

X = np.array([[-1 ,1],[-1, 1]])
Y = np.array([[ 1 ,1],[-1,-1]])
Z = np.array([[-1,-1],[-1,-1]])

ground = ax.plot_surface(3* X, 3*Y, Z, linewidth=0, color='g')

verts = [ [-1, 2, 0],
          [-1,-2, 0],
          [ 1, 2, 0],
          [ 1,-2, 0]]

def plot_surface(verts):
    l = np.array(verts).T
    x = l[0].reshape((2,2))
    y = l[1].reshape((2,2))
    z = l[2].reshape((2,2))
    # print(x)
    # print(y)

    return ax.plot_surface(x, y, z, linewidth=0, color='r')

s = plot_surface(  verts )

# Customize the z axis.
ax.set_zlim(-1.01, 1.01)

a = 0
while 1:
    a += 0.01
    q = Quaternion.fromAxisAngle( [0,0,1], a )
    s.remove()
    s = plot_surface( [q.rotate(v) for v in verts] )
    plt.pause(0.01)


# plt.show()
