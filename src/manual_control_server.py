#!/usr/bin/env python3

import rospy
import actionlib
from roboat_pkg.msg import ManualControlAction
from std_msgs.msg import Float32, Bool

class ManualControlAction:

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, ManualControlAction, execute_cb=self.execute_cb, auto_start = False)

        rospy.Subscriber('/user/throttle', Float32, self.user_throttle_cb)
        self._throttle = 0.0 # Default Value for Throttle

        self._thrust_vec = rospy.Publisher('/thrust/vec', Float32, queue_size=1)

        self._as.start()

    def user_throttle_cb(self, data):
        self._throttle = data.data
    
    def execute_cb(self, goal):
        r = rospy.Rate(0.5)

        # Reset Throttle, so we don't use previous value
        self._throttle = 0.0
         
        # publish info to the console for the user
        rospy.loginfo(f"{self._action_name}: Starting")
           
        while True:
            if self._as.is_preempt_requested():
                self._thrust_vec.publish(0.0)
                rospy.loginfo(f"{self._action_name}: Preempted")

                self._as.set_preempted()
                break

            self._thrust_vec.publish(self._throttle)
        
            r.sleep()
         

if __name__ == '__main__':
    rospy.init_node('manual_control_server')
    server = ManualControlAction(rospy.get_name())
    rospy.spin()