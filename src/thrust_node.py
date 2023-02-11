#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool
from drivers import Rotor 

class Thrust:

    def __init__(self):
        rospy.init_node('thrust')

        rotor_channel1 = rospy.get_param("thrust/rotor_channel1")
        rotor_channel2 = rospy.get_param("thrust/rotor_channel2")

        self.rotor1 = Rotor(rotor_channel1)
        self.rotor2 = Rotor(rotor_channel2)
        
        rospy.Subscriber("thrust_vec_out", Float32, self.rotor_power_hander)
        rospy.Subscriber("e_stop", Bool, self.e_stop_hander)

        self.e_stop = True

        rospy.loginfo("Thrust Initialized!")

        rospy.spin()

    def rotor_power_hander(self, data):
        if self.e_stop is True:
            self.rotor1.off()
            self.rotor2.off()

        power = data.data
        self.rotor1.set(power)
        self.rotor2.set(power)

    def e_stop_hander(self, data):
        if data.data is True:
            self.rotor1.off()
            self.rotor2.off()
            self.e_stop = True
        else:
            self.e_stop = False
    
   

if __name__ == '__main__':
    try:
        Thrust()
    except rospy.ROSInterruptException:
        pass