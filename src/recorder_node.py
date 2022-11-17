#!/usr/bin/env python3

import rospy
from roboat_pkg.msg import GPSInfo
import csv

def log_gps(data):
    with open('/home/beagle/gps.csv','a', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([data.measurementTime, data.groundSpeed, data.lon, data.lat])
    
def listener():

    rospy.init_node('recorder')

    rospy.Subscriber("gps_info", GPSInfo, log_gps)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()