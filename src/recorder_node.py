#!/usr/bin/env python3

import rospy
from roboat_pkg.msg import GPSInfo
import csv
class Recorder:

    def __init__(self):
        rospy.init_node('recorder')

        self.csv_encoding = rospy.get_param("recorder/csv_encoding")
        self.gps_csv_path = rospy.get_param("recorder/gps_csv_path")
        
        rospy.Subscriber("gps_info", GPSInfo, self.log_gps)

        rospy.loginfo("Recorder Initialized!")

        rospy.spin()

    def log_gps(self, data):
        with open(self.gps_csv_path,'a', newline='', encoding=self.csv_encoding) as f:
            writer = csv.writer(f)
            writer.writerow([data.measurementTime, data.groundSpeed, data.lon, data.lat])
    
   

if __name__ == '__main__':
    Recorder()