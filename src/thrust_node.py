#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool
from drivers import Rotor 

class Thrust:

    def __init__(self):
        rospy.init_node('thrust')

        rotor_channel = rospy.get_param("thrust/rotor_channel")

        self.rotor = Rotor(rotor_channel)
        
        rospy.Subscriber("thrust_vec_out", Float32, self.rotor_power_hander)
        rospy.Subscriber("e_stop", Bool, self.e_stop_hander)

        self.e_stop = True

        rospy.loginfo("Thrust Initialized!")

        rospy.spin()

    def rotor_power_hander(self, data):
        if self.e_stop is True:
            self.rotor.off()

        power = data.data
        self.rotor.set(power)

    def e_stop_hander(self, data):
        if data.data is True:
            self.rotor.off()
            self.e_stop = True
        else:
            self.e_stop = False
    
   

if __name__ == '__main__':
    thrust = None
    try:
        Thrust()
    except rospy.ROSInterruptException:
        pass