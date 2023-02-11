#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool
import tty, termios
import sys, select, os
class KeyboardController:

    def __init__(self):
        # logging the node start
        rospy.loginfo("Starting node keyboard_controller")

        # Initialise the node
        rospy.init_node("keyboard_controller", anonymous=True)

        # Create a publisher to the keypress topic
        self.thrust_publisher = rospy.Publisher('/thrust', Float32, queue_size=1)
        self.e_stop_publisher = rospy.Publisher('/e_stop', Bool, queue_size=1)

        rate = rospy.Rate(10)


    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        return key
    def set_e_stop(self, isEStopEnabled):
        self.e_stop_publisher.publish(Bool(isEStopEnabled))

    def set_thrust(self, thrust_val):
        self.thrust_publisher.publish(Float32(thrust_val))


if __name__ == '__main__':
    try:
        kc = KeyboardController()
        
        print("Keyboard Controller Enabled!\n--------------------------------\n\n")
        print("q = Quit\ne = turn off e_stop\n<Space> = Turn on E_stop\n1-10 = Speed of rotor")

        while not rospy.is_shutdown():
            key = kc.getKey()
            if key == 'e':
                kc.set_e_stop(False)
                print("E-Stop Disabled!")
            elif key == 'q':
                break
            elif key == ' ':
                kc.set_e_stop(True)
                print("E-Stop Enabled!")
            elif key.isnumeric():
                number = int(key)
                kc.set_thrust(number * 0.1)
                print(f"Thrust set to {number * 0.1}")
            

        
        # Shutdown
        kc.set_e_stop(True)
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Stopping keyboard_publisher")
        kc.set_e_stop(True)