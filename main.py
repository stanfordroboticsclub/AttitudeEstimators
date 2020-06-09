from quaternion import Quaternion
from imu import AndroidIMU, IPhoneIMU
from filters import ComplementaryFilter, MEKF
from displays import StaticDisplay, DynamicDisplay
from argparse import ArgumentParser


if __name__ == "__main__":

    parser = ArgumentParser()
    parser.add_argument('--imu', help='type of IMU')
    parser.add_argument('--filter_type', help='filter to run')
    parser.add_argument('--display_euler', help='whether final plot should use Euler angles instead of quat', action='store_true')
    args = parser.parse_args()
    
    if args.imu.lower() == 'android':
        imu = AndroidIMU()
    elif args.imu.lower() == 'iphone':
        imu = IPhoneIMU()
    else:
        error("'{}' not supported!".format(args.imu))

    if args.filter_type == 'mekf':
        filt = MEKF()        
    elif args.filter_type == 'complementary':
        filt = ComplementaryFilter()
    else:
        error("'{}' not supported!".format(args.filter_type))

    dynamic_display = DynamicDisplay()
    static_display = StaticDisplay(quat=not args.display_euler)
    
    while 1:
        try:
            filt.update_gyro( *imu.get_gyro_vect() )
            filt.update_acel( imu.get_acel_vect() )
            dynamic_display.plot_quat( filt.quat() )
            orientation = imu.get_orientation()
            if not args.display_euler:
                static_display.add_data(imu.last_time, filt.quat().q, Quaternion.fromEuler(*orientation).q)
            else:
                display.add_data(imu.last_time, filt.quat().toEuler(), orientation)

        except KeyboardInterrupt as e:
            static_display.show()
            exit(0)


        
