#!/usr/bin/env python3
import rospy
from actionlib import SimpleActionClient
from roboat_pkg.msg import ManualControlAction, ManualControlGoal
from std_msgs.msg import String, Bool

class Commander:
    def __init__(self):
        rospy.Subscriber('/user/command', String, self.command_cb)
        rospy.Subscriber('/e_stop', Bool, self.e_stop_cb)

        self._e_stop = True # E_Stop Default On

        self._manual_client = SimpleActionClient("manual_control", ManualControlAction)
        self._manual_client.wait_for_server()

    def e_stop_cb(self, data):
        self._e_stop = data.data
        if data.data:
            self.turn_off_everything()

    def command_cb(self, data):
        command = data.data

        # Stop everything for e-stop
        if self._e_stop:
            command = "STOP"

        if command == "MANUAL":
            self._manual_client.send_goal(ManualControlGoal())
        elif command == "STOP":
            self.turn_off_everything()
    
    def turn_off_everything(self):
        self._manual_client.cancel_all_goals()

if __name__ == '__main__':
    rospy.init_node('commander')
    Commander()
    rospy.spin()