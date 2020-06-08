from display import IMU, MEKF, ComplementaryFilter
from UDPComms import Publisher
import time
from mpu6050 import mpu6050
import numpy as np


def dictvec_to_array(d):
    return np.array([d['x'], d['y'], d['z']])

def clean_sensor_data(gyro, accel, bias):
    gyro = dictvec_to_array(gyro)
    gyro = (gyro - bias) / 180.0 * np.pi
    accel = -dictvec_to_array(accel)
    return gyro, accel


if __name__ == "__main__":
    imu = IMU()
    filt_comp = ComplementaryFilter(alpha=0.01)
    filt_mekf = MEKF(Q_gyro=1e-6, Q_bias=1e-12, R=1)

    comp_pub = Publisher(8007)
    mekf_pub = Publisher(8008)
    sensor = mpu6050(0x68)
    sensor.set_gyro_range(mpu6050.GYRO_RANGE_2000DEG)

    DT = 0.005

    last_update = time.time()
    t = 0
    bias = dictvec_to_array(sensor.get_gyro_data())
    while 1:
        now = time.time()
        
        if now - last_update > DT:
            gyro_data = sensor.get_gyro_data()
            accel_data = sensor.get_accel_data()
            gyro_data, accel_data = clean_sensor_data(gyro_data, accel_data, bias)
            
            filt_comp.update_gyro(gyro_data, DT)
            filt_comp.update_acel(accel_data)
            
            filt_mekf.update_gyro(gyro_data, DT)
            filt_mekf.update_acel(accel_data)
                
            comp_pub.send(list(filt_comp.quat().q))
            mekf_pub.send(list(filt_mekf.quat().q))

            if t % 10 == 0:
                print(now - last_update, gyro_data, filt_comp.quat())

            last_update = now
            t += 1
