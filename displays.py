from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import time


class StaticDisplay:
    def __init__(self, quat=True, label1="filtered", label2=None):
        self.dim = 4 if quat else 3
        if quat:
            self.axis_names = ["qw", "qx", "qy", "qz"]
        else:
            self.axis_names = ["yaw", "pitch", "roll"]
        self.start_time = time.time()
        self.timestamps = []
        self.filtered = []
        self.filtered2 = []
        self.gt = []

        self.label1 = label1
        self.label2 = label2

    def add_data(self, timestamp, filtered, gt, filtered2=None):
        self.timestamps.append(timestamp - self.start_time)
        self.filtered.append(filtered)
        self.gt.append(gt)
        if filtered2 is not None:
            self.filtered2.append(filtered2)

    def show(self, axis=None):
        self.dim = 1 if axis is not None else self.dim
        self.fig, self.ax = plt.subplots(self.dim, 1)

        self.filtered = np.array(self.filtered)
        self.filtered2 = np.array(self.filtered2)
        self.gt = np.array(self.gt)

        if axis is None:
            for i in range(self.dim):
                self.ax[i].plot(self.timestamps, self.filtered[:, i], label=self.label1)
                self.ax[i].plot(self.timestamps, self.gt[:, i], label="ground truth")
                if len(self.filtered2) > 0:
                    self.ax[i].plot(
                        self.timestamps, self.filtered2[:, i], label=self.label2
                    )
                self.ax[i].set_ylabel(self.axis_names[i])
                self.ax[i].legend()
        else:
            self.ax.plot(self.timestamps, self.filtered[:, axis], label=self.label1)
            self.ax.plot(self.timestamps, self.gt[:, axis], label="ground truth")
            if len(self.filtered2) > 0:
                self.ax.plot(
                    self.timestamps, self.filtered2[:, axis], label=self.label2
                )
            self.ax.set_ylabel(self.axis_names[axis])
            self.ax.set_xlabel("time (s)")
            self.ax.legend()

        plt.show()


class DynamicDisplay:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.gca(projection="3d")

        X = np.array([[-1, 1], [-1, 1]])
        Y = np.array([[1, 1], [-1, -1]])
        Z = np.array([[-1, -1], [-1, -1]])

        scale = 3
        self.ground = self.ax.plot_surface(
            scale * X, scale * Y, scale * Z, color="g", zorder=1
        )
        bounds = scale + 0.01
        self.ax.set_zlim(-bounds, bounds)

        self.verts = [[-1, 2, 0], [-1, -2, 0], [1, 2, 0], [1, -2, 0]]
        self.arrow = [0, 0, 1]
        self.s = None
        self.a = None
    def plot_quat(self, q):
        verts = [q.rotate(v) for v in self.verts]
        l = np.array(verts).T
        x = l[0].reshape((2, 2))
        y = l[1].reshape((2, 2))
        z = l[2].reshape((2, 2))

        if self.s != None:
            self.s.remove()
        if self.a != None:
            self.a.remove()

        self.s = self.ax.plot_surface(
            x, y, z, linewidth=0, color="r", zorder=2, alpha=0.8
        )
        arrow = q.rotate(self.arrow)
        self.a = self.ax.quiver(
            0, 0, 0, *arrow, color="blue", alpha=0.8, lw=3, zorder=3
        )

        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.set_zlabel("z")
        plt.pause(0.01)
