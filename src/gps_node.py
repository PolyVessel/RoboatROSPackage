#!/usr/bin/env python3
import rospy
from roboat_pkg.msg import GPSInfo

from drivers import GPS, GPSNoSignal
from drivers.util import TimeoutException

def configure_gps():
    serial_port = rospy.get_param('gps/serial_port')
    timeout = rospy.get_param('gps/timeout')
    return GPS(serial_port, timeout)

def get_gps_data(gps):
    try:
        return gps.poll_sensor()

    # Critical, Expected Errors
    except TimeoutException as e:
        rospy.logfatal_throttle(180, "GPS Module Not Connected")
    except GPSNoSignal as e:
        rospy.log_throttle(180, "No GPS Signal")

    # More serious errors, that would not be expected
    except ValueError as e:
        rospy.logfatal_throttle_identical(120, "Value Error thrown by GPS node! Details: " + str(e))
    except IOError as e:
        rospy.logfatal_throttle_identical(120, "IO Error thrown by GPS node! Details: " + str(e))

def gps_location_publisher():
    gps_pub = rospy.Publisher('gps_info', GPSInfo, queue_size=5)
    rospy.init_node('gps')
    rate = rospy.Rate(0.5) # 0.5hz
    
    gps = configure_gps()

    rospy.loginfo("GPS Initialized")
    
    while not rospy.is_shutdown():
        data = get_gps_data(gps)
        if data is not None:
            gps_info = GPSInfo()
            gps_info.lon = data.lon
            gps_info.lat = data.lat
            gps_info.groundSpeed = data.gSpeed
            gps_info.measurementTime = rospy.Time.now()

            gps_pub.publish(gps_info)
           
        rate.sleep()

if __name__ == '__main__':
    try:
        gps_location_publisher()
    except rospy.ROSInterruptException:
        pass

