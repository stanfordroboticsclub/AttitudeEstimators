from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from displays import DynamicDisplay, StaticDisplay
from quaternion import Quaternion
import UDPComms
import time
import numpy as np
from argparse import ArgumentParser
import serial

def get_angle(ser):
    reading = ser.readline().decode().split('\t')
    theta = int(reading[1]) / 8192 * 2 * np.pi
    return theta

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('--filter_type', help='type of IMU')
    args = parser.parse_args()
    
    # dynamic_display = DynamicDisplay()
    static_display = StaticDisplay(quat=False, label1="mekf", label2="complementary")

    filter_sub = None
    secondary_sub = None
    if args.filter_type.lower() == "mekf":
        filter_sub = UDPComms.Subscriber(8008)
    elif args.filter_type.lower() == "comp":
        filter_sub = UDPComms.Subscriber(8007)
    elif args.filter_type.lower() == "both":
        filter_sub = UDPComms.Subscriber(8008) # mekf
        secondary_sub = UDPComms.Subscriber(8007) #comp
    else:
        error("{} not supported".format(args.filter_type))
    time.sleep(0.5)

    ser = serial.Serial('/dev/cu.usbmodem71393001', timeout=0.1)

    bias = get_angle(ser) - Quaternion.toEuler(Quaternion(filter_sub.get()))[2]
    while  1:
        try:
            theta = get_angle(ser) - bias
            q = Quaternion(filter_sub.get())
            q2 = None
            if secondary_sub is not None:
                q2 = Quaternion(secondary_sub.get())
            q_gt = Quaternion.fromAxisAngle(np.array([1, 0, 0]), theta)
            static_display.add_data(time.time(), Quaternion.toEuler(q), Quaternion.toEuler(q_gt), Quaternion.toEuler(q2))
            print(time.time(), Quaternion.toEuler(q_gt))

        except KeyboardInterrupt as e:
            static_display.show(axis=2)
            exit(0)
