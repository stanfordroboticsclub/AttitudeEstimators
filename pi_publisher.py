from UDPComms import Publisher
import time
from mpu6050 import mpu6050

pub = Publisher(8007)
sensor = mpu6050(0x68)

last_send = time.time()

bias = sensor.get_gyro_data()
while 1:
    now = time.time()
    if (now - last_send >= 0.01):
        accel = sensor.get_accel_data()
        accel_done = time.time()
        gyro = sensor.get_gyro_data()
        gyro_done = time.time()
        gyro = {k: (v-bias[k])/180.0*3.14159 for k, v in gyro.items()}
        accel = {k: -v for k, v in accel.items()}

        pub.send((accel['x'], accel['y'], accel['z'], gyro['x'], gyro['y'], gyro['z']))
       
        print('{:<1.3f}\t{:<1.3f}\t{:<1.3f}'.format(gyro['x'],gyro['y'],gyro['z']))
        #print('{:1.3f}\t{:1.3f}\t{:1.3f}'.format(accel['x'],accel['y'],accel['z']))
        #print(round(gyro['x'],3), '\t', round(gyro['y'], 3), '\t', round(gyro['z'],3))
        #print(now - last_send, accel_done - now, gyro_done - accel_done)
        last_send = now
