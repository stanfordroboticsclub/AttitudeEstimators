# from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import UDPComms
import time


def gather_data():
    sub = UDPComms.Subscriber(8007, timeout=0.5)

    N = 1000
    data = np.zeros((N, 6))

    i = 0
    for i in range(N):
        ax, ay, az, gx, gy, gz = sub.recv()
        data[i, :] = [ax, ay, az, gx, gy, gz]
        if i % 10 == 0:
            print(i)

    np.save("data.npy", data)


def covariances():
    data = np.load("data.npy")

    np.set_printoptions(precision=3, suppress=False)
    accel_cov = np.cov(data[:, :3].T)
    print("\nAccelerometer covariance")
    print(accel_cov)

    np.set_printoptions(precision=3, suppress=False)
    print("\nGyro covariance")
    gyro_cov = np.cov(data[:, 3:].T)
    print(gyro_cov)

    plt.figure()
    plt.subplot(211)
    plt.imshow(gyro_cov)
    plt.title("Gyro Cov")

    plt.subplot(212)
    plt.imshow(accel_cov)
    plt.title("Accel Cov")
    plt.show()


def plot_data():
    data = np.load("data.npy")

    # norm = np.linalg.norm(data, axis=1)
    # print(norm)

    plt.figure()
    o = plt.plot(data[:, :3] - data[:, :3].mean(axis=0), linewidth=0.75)
    plt.legend(o, ("x", "y", "z"))
    plt.title("Centered Accelerometer Data")
    plt.ylabel("Centered Acceleration [m/s/s]")
    plt.xlabel("Time [0.01s]")
    plt.show()

    plt.figure()
    o = plt.plot(data[:, 3:] - data[:, 3:].mean(axis=0), linewidth=0.75)
    plt.legend(o, ("x", "y", "z"))
    plt.title("Centered Gyro Data")
    plt.ylabel("Centered Gyro [rad/s]")
    plt.xlabel("Time [0.01s]")
    plt.show()


if __name__ == "__main__":
    # gather_data()
    # plot_data()
    covariances()
