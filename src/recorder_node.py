#!/usr/bin/env python3

import rospy
from roboat_pkg.msg import GPSInfo, radio_telemetry
from std_msgs.msg import String, Bool
import json
import csv
class Recorder:

    def __init__(self):
        self.telemetry_dict = {}

        rospy.init_node('recorder')


        self.csv_encoding = rospy.get_param("/recorder/csv_encoding")
        self.storage_path = rospy.get_param("/recorder/storage_path")
        self.period = rospy.get_param("/recorder/transmit_period")
        rospy.Subscriber("/sensor/gps", GPSInfo, self.log_gps)
        rospy.Subscriber("e_stop", Bool, self.log_estop)
        rospy.Subscriber("radio_telemetry", radio_telemetry, self.log_depth)
        self.tansmitter = rospy.Publisher("radio_transmit", String, queue_size=5)

        rospy.loginfo("Recorder Initialized!")

        while not rospy.is_shutdown():
            self.transmit_telemetry()
            rospy.sleep(self.period)
        
        rospy.spin()

    def log_gps(self, data):
        self.telemetry_dict["GPS"] = {}
        self.telemetry_dict["GPS"]["time"] = data.measurementTime
        self.telemetry_dict["GPS"]["ground_speed"] = data.groundSpeed
        self.telemetry_dict["GPS"]["longitude"] = data.lon
        self.telemetry_dict["GPS"]["latitude"] = data.lat
    
    def log_estop(self, data):
        self.telemetry_dict["E-Stop"] = data.value
    
    def log_radio(self, data):
        self.telemetry_dict["Radio"] = {}
        self.telemetry_dict["Radio"]["missing-packets"] = self.missing_packets.copy()
        self.telemetry_dict["Radio"]["last-packet-received"] = data.last_packet_received

    def transmit_telemetry(self):
        telemetry_string = json.dumps(self.telemetry_dict)
        self.tansmitter.publish(telemetry_string)
        self.store_telemetry(telemetry_string)
    
    def store_telemetry(self, telemetry_string):
        path = self.storage_path + "telemetry_" + str(rospy.get_time()) + ".json"
        with open(path, 'w') as f:
            f.write(telemetry_string)
    

if __name__ == '__main__':
    try:
        Recorder()
    except rospy.ROSInterruptException:
        pass