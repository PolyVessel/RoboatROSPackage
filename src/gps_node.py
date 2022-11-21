#!/usr/bin/env python3
import rospy
from roboat_pkg.msg import GPSInfo

from drivers import GPS, GPSNoSignal
from drivers.util import TimeoutException

class GPSNode():

    def __init__(self):
        gps_pub = rospy.Publisher('gps_info', GPSInfo, queue_size=5)
        rospy.init_node('gps')

        gps_poll_rate = rospy.get_param("gps/poll_rate")

        rate = rospy.Rate(gps_poll_rate) # 0.5hz
        
        gps = self.configure_gps()

        rospy.loginfo("GPS Initialized")
        
        while not rospy.is_shutdown():
            data = self.get_gps_data(gps)
            if data is not None:
                gps_info = GPSInfo()
                gps_info.lon = data.lon
                gps_info.lat = data.lat
                gps_info.groundSpeed = data.gSpeed
                gps_info.measurementTime = rospy.Time.now()

                gps_pub.publish(gps_info)
            
            rate.sleep()

    def configure_gps(self):
        serial_port = rospy.get_param('gps/serial_port')
        timeout = rospy.get_param('gps/timeout')
        return GPS.fromgpsconfig(serial_port, timeout)

    def get_gps_data(self, gps):
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


if __name__ == '__main__':
    try:
        GPSNode()
    except rospy.ROSInterruptException:
        pass

