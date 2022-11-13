#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensors import GPS

def gps_location_publisher():
    pos_lat = rospy.Publisher('GPS_position_lat', Float64, queue_size=5)
    pos_lon = rospy.Publisher('GPS_position_lon',Float64, queue_size=5)
    vel_pub = rospy.Publisher('GPS_velocity', Float64, queue_size=5)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.5) # 0.5hz
    gps = GPS()
    while not rospy.is_shutdown():
        data = gps.poll_sensor()
        pos_lon.publish(data.lon)
        pos_lat.publish(data.lat)
        vel_pub.publish(data.gSpeed)
        rospy.loginfo("Lon: {}, Lat: {}".format(data.lon, data.lat))
        rate.sleep()

if __name__ == '__main__':
    try:
        gps_location_publisher()
    except rospy.ROSInterruptException:
        pass

