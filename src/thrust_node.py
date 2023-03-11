#!/usr/bin/env python3

import rospy
from roboat_pkg.msg import Thrust, Bool
from drivers import Rotor 

class Thrust:

    def __init__(self):
        rospy.init_node('thrust')
        
        rospy.loginfo("Thrust Initializing...")

        rotor_serial_port = rospy.get_param("/thrust/rotor_serial_port")

        rospy.loginfo("Got Parameters!")

        self.rotor = Rotor(rotor_serial_port)

        
        rospy.loginfo("Initialized Rotors!")
        
        rospy.Subscriber("/thrust/vec", Thrust, self.rotor_power_hander)
        rospy.Subscriber("/e_stop", Bool, self.e_stop_hander)

        self.e_stop = True

        rospy.loginfo("Thrust Initialized!")

        rospy.spin()

    def rotor_power_hander(self, data):
        if self.e_stop is True:
            self.rotor.off()

        self.rotor.set(data.dutyLeft, data.DutyRight)

    def e_stop_hander(self, data):
        if data.data is True:
            self.rotor.off()
            self.e_stop = True
        else:
            self.e_stop = False
    
   

if __name__ == '__main__':
    try:
        Thrust()
    except rospy.ROSInterruptException:
        pass