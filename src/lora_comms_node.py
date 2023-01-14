#!/usr/bin/env python3

from drivers.radio import Radio, RadioResponseBad
from drivers.Depacketizer import Depacketizer
from drivers.Packet import Packet
import rospy
from std_msgs.msg import Bool
from roboat_pkg.msg import telemetry as telemetry_msg

def comms_node():
    rospy.init_node('lora_comms')
    radio = configure_radio()
    poll_rate = rospy.get_param("radio/poll_rate")
    rate = rospy.Rate(poll_rate)
    test_radio(radio)
    depacketizer = Depacketizer()
    while not rospy.is_shutdown():
        depacketizer.write(radio.receive())
        valid_packets = depacketizer.read_packets_from_buffer()
        for packet in valid_packets:
            rospy.loginfo(packet)
        rate.sleep()


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

if __name__ == "__main__":
    try:
        comms_node()
    except rospy.ROSInterruptException:
        pass