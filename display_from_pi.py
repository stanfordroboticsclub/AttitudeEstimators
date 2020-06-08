from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt 

from display import Display, Quaternion 
import UDPComms
import time
import numpy as np

if __name__ == "__main__":
    display = Display()

    t = 0

    mekf_sub = UDPComms.Subscriber(8008)
    comp_sub = UDPComms.Subscriber(8007)
    time.sleep(0.5)

    while 1:
        o = mekf_sub.get()
        print(o)
        mekf_q = Quaternion(mekf_sub.get())
        comp_q = Quaternion(comp_sub.get())

        display.plot_quat(mekf_q)
        # display.plot_quat(comp_q)
        
        plt.pause(0.01)
        t += 1