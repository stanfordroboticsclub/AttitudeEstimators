from display import IMU, MEKF, ComplementaryFilter
from UDPComms import Publisher
import time
from mpu6050 import mpu6050
import numpy as np


def clean_sensor_data(gyro, accel):
    gyro = {k: (v - bias[k]) / 180.0 * np.pi for k, v in gyro.items()}
    accel = {k: -v for k, v in accel.items()}
    return gyro, accel


if __name__ == "__main__":
    imu = IMU()
    filt_comp = ComplementaryFilter()
    filt_mekf = MEKF()

    pub = Publisher(8007)
    sensor = mpu6050(0x68)

    DT = 0.01

    while 1:
        now = time.time()

        if now - last_update > DT:
            gyro_data = sensor.get_gyro_data()
            accel_data = sensor.get_accel_data()
            filt_comp.update_gyro(gyro_data, DT)
            filt_comp.update_acel(accel_data)

            last_update = now

            t += 1
