from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from displays import DynamicDisplay, StaticDisplay
from quaternion import Quaternion
import UDPComms
import time
import numpy as np
from argparse import ArgumentParser


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('--filter_type', help='type of IMU')
    args = parser.parse_args()
    
    dynamic_display = DynamicDisplay()
    static_display = StaticDisplay(quat=False)

    filter_sub = None
    if args.filter_type.lower() == "mekf":
        filter_sub = UDPComms.Subscriber(8008)
    elif args.filter_type.lower() == "comp":
        filter_sub = UDPComms.Subscriber(8007)
    else:
        error("{} not supported".format(args.filter_type))
    time.sleep(0.5)

    while 1:
        try:
            q = Quaternion(filter_sub.get())
            dynamic_display.plot_quat(q)
            static_display.add_data(time.time(), q.q, np.zeros(4))

        except KeyboardInterrupt as e:
            static_display.show()
            exit(0)
