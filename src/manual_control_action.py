#!/usr/bin/env python3

import rospy
import actionlib
from roboat_pkg.msg import GPSInfo
from std_msgs.msg import Float32, Bool

class ManualControlAction:

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, roboat_pkg.msg.ManualControlAction, execute_cb=self.execute_cb, auto_start = False)

        rospy.Subscriber('/user/throttle', Float32, self.user_throttle_cb)
        self._throttle = 0.0 # Default Value for Throttle

        self._thrust_vec = rospy.Publisher('/thrust/vec', Float32, queue_size=1)

        self._as.start()

    def user_throttle_cb(self, data):
        self._throttle = data.data
    
    def execute_cb(self, goal):
        # helper variable
        r = rospy.Rate(0.5)
         
        # publish info to the console for the user
        rospy.loginfo(f"{self._action_name}: Starting")
           
        while True:
            if self._as.is_preempt_requested():
                rospy.loginfo(f"{self._action_name}: Preempted")
                break

            self._thrust_vec.publisher(self._throttle)
        
            r.sleep()
         