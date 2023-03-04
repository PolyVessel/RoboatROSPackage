#!/usr/bin/env python3
from radio import Radio, RadioResponseBad

from Depacketizer import Depacketizer
from Packet import Packet
import rospy
from std_msgs.msg import Bool
from roboat_pkg.msg import telemetry as telemetry_msg


def configure_radio():
    serial_port = rospy.get_param('/lora_radio/serial_port')
    m0_pin = rospy.get_param('/lora_radio/m0_pin')
    m1_pin = rospy.get_param('/lora_radio/m1_pin')
    aux_pin = rospy.get_param('/lora_radio/aux_pin')

    return Radio(serial_port, m0_pin, m1_pin, aux_pin)


def comms_node():
    rospy.init_node('lora_comms')

    radio = configure_radio()
    poll_rate = rospy.get_param("lora_radio/poll_rate")
    e_stop_pub = rospy.Publisher('e_stop', Bool, queue_size=1)
    command_dispatch_pub = rospy.Publisher('command_dispatch', telemetry_msg, queue_size=1)
        
    rate = rospy.Rate(poll_rate)

    depacketizer = Depacketizer()
    rospy.loginfo("Radio Initialized, ready to listen")
    while not rospy.is_shutdown():
        depacketizer.write(radio.receive())
        valid_packets = depacketizer.read_packets_from_buffer()
        for packet in valid_packets:
            rospy.loginfo(packet)
            command_dispatch_pub.publish(packet)
        rate.sleep()



if __name__ == "__main__":
    try:
        comms_node()
    except rospy.ROSInterruptException:
        pass