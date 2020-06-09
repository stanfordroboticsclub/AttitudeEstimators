import time
from quaternion import Quaternion
import numpy as np

class IPhoneIMU:
    
    def __init__(self):
        import UDPComms
        
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
        return Quaternion.fromGyro( [gx,gy,gz], dt)

    def get_gyro_vect(self):
        try:
            ax,ay,az, gx, gy, gz = self.sub.get()
        except UDPComms.timeout:
            return [0,0,0], 0
        dt = time.time() - self.last_time
        print(dt)
        self.last_time = time.time()
        return [gx, gy, gz], dt

    def get_acel_vect(self):
        try:
            ax,ay,az, gx, gy, gz = self.sub.get()
        except UDPComms.timeout:
            return None

        return np.array([ax, ay, az])

class AndroidIMU:
    """ Interpret UDP packets sent by Sensor UDP app (https://play.google.com/store/apps/details?id=com.ubccapstone.sensorUDP&hl=en_US)
    """
    
    def __init__(self):
        import UDPComms
        import struct
        
        def decode(data):
            vals = []
            for val in struct.iter_unpack('>f', data):
                vals.append(val[0])
            return vals
        
        self.sub = UDPComms.Subscriber(8007, decode_func=decode)
        self.last_time = time.time()
        self.timeout = UDPComms.timeout
        self.accel_offset = 0
        self.gyro_offset = 6
        self.orientation_offset = 9
        time.sleep(0.25)

    def get_gyro_quat(self):
        try:
            data = self.sub.get()
        except self.timeout:
            print("Timeout!")
            return Quaternion.fromAxisAngle( [0,0,1], 0)
        dt = time.time() - self.last_time
        print(dt)
        self.last_time = time.time()
        return Quaternion.fromGyro( data[gyro_offset:gyro_offset+3], dt)

    def get_gyro_vect(self):
        try:
            data = self.sub.get()
        except self.timeout:
            print("Timeout!")
            return [0,0,0], 0
        dt = time.time() - self.last_time
        print("dt: ", dt)
        self.last_time = time.time()
        return data[self.gyro_offset:self.gyro_offset+3], dt

    def get_acel_vect(self):
        try:
            data = self.sub.get()
        except self.timeout:
            return None

        acel = np.array(data[self.accel_offset:self.accel_offset+3])
        acel -= np.array([0,0,-9.81])
        acel /= 9.81
        return acel

    def get_orientation(self):
        try:
            data = self.sub.get()
        except self.timeout:
            return None

        # in order azimuth, pitch, roll
        euler_angle = np.array(data[self.orientation_offset:self.orientation_offset+3])
        print(euler_angle)
        return euler_angle * np.pi/180
