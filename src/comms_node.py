#!/usr/bin/env python3

from drivers.radio import Radio, RadioResponseBad
import rospy
from std_msgs.msg import Bool
from roboat_pkg.msg import telemetry as telemetry_msg

def configure_radio():
    serial_port = rospy.get_param('lora_radio/serial_port')
    m0_pin = rospy.get_param('lora_radio/m0_pin')
    m1_pin = rospy.get_param('lora_radio/m1_pin')
    aux_pin = rospy.get_param('lora_radio/aux_pin')

    return Radio(serial_port, m0_pin, m1_pin, aux_pin)

def test_radio(radio, e_stop_pub):
    try:
        ping_data = radio.ping_radio()
        rospy.loginfo("Radio Passed Self-Test! Returning Data: " + str(ping_data))
    except RadioResponseBad as e:
        rospy.logfatal("Radio self-test failed! Reason: " + str(e))
        e_stop_state = Bool()
        e_stop_state.data = True
        e_stop_pub.publish(e_stop_state)


def comms_node():
    e_stop_pub = rospy.Publisher('e_stop', Bool, queue_size=1)
    telemetry_listener = rospy.Subscriber('telemetry_logs', telemetry_msg)
    rospy.init_node('comms')
    
    radio = configure_radio()
    test_radio(radio, e_stop_pub)
    rospy.spin()



if __name__ == "__main__":
    try:
        comms_node()
    except rospy.ROSInterruptException:
        pass