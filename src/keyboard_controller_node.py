#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from roboat_pkg.msg import Thrust
class KeyboardController:

    def __init__(self):
        # logging the node start
        rospy.loginfo("Starting node keyboard_controller")

        # Initialise the node
        rospy.init_node("keyboard_controller", anonymous=True)

        # Create a publisher to the keypress topic
        self.thrust_publisher = rospy.Publisher('/thrust/vec', Thrust, queue_size=1)
        self.e_stop_publisher = rospy.Publisher('/e_stop', Bool, queue_size=1)

        rate = rospy.Rate(10)


    def getKey(self):
        return input("> ")
    def set_e_stop(self, isEStopEnabled):
        self.e_stop_publisher.publish(Bool(isEStopEnabled))

    def set_thrust(self, left, right):
        t = Thrust()
        t.dutyLeft = left
        t.dutyRight = right

        self.thrust_publisher.publish(t)


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
            elif any(char.isdigit() for char in key):
                try:
                    split = key.split(" ")
                    left = int(split[0])
                    right = int(split[1])
                    kc.set_thrust(left, right)
                    print(f"Thrust set to {left}/180 and {right}/180")
                except:
                    print("Invalid input")

            

        
        # Shutdown
        kc.set_e_stop(True)
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Stopping keyboard_publisher")
    finally:
        kc.set_e_stop(True)
