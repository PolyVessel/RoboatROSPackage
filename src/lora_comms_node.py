#!/usr/bin/env python3
from drivers.radio import Radio, RadioResponseBad

from drivers.Depacketizer import Depacketizer
from drivers.Packet import Packet
import rospy
from std_msgs.msg import Bool, String
from roboat_pkg.msg import commanding, radio_telemetry



class ComsNode:
   
    def __init__(self):
        rospy.init_node('lora_comms')
        self.last_packet_sent = 0
        self.last_packet_received = 0
        self.missing_packets = []
        self.radio = self.configure_radio()

        poll_rate = rospy.get_param("lora_radio/poll_rate")
        command_dispatch_pub = rospy.Publisher('command_dispatch', commanding, queue_size=1)
        self.elemetry_pub = rospy.Publisher('telemetry', radio_telemetry, queue_size=1)
        rospy.Subscriber("radio_transmit", String, self.transmit_callback)

        
        rate = rospy.Rate(poll_rate)

        depacketizer = Depacketizer()
        rospy.loginfo("Radio Initialized, ready to listen")
        cycles = 0
        while not rospy.is_shutdown():
            depacketizer.write(self.radio.receive())
            valid_packets = depacketizer.read_packets_from_buffer()
            for packet in valid_packets:
                if packet.packet_id != self.last_packet_received + 1:
                    self.missing_packets.append(packet.packet_id + 1)
                    rospy.logwarn("Packet loss detected, last packet received: " + str(self.last_packet_received) + " current packet: " + str(packet.packet_id))
                self.last_packet_received = packet.packet_id
                rospy.loginfo(packet)
                command_dispatch_pub.publish(packet)
            cylces += 1
            if cycles % 15 == 0:
                self.gather_telemetry()
            rate.sleep()

    def configure_radio(self):
        serial_port = rospy.get_param('/lora_radio/serial_port')
        m0_pin = rospy.get_param('/lora_radio/m0_pin')
        m1_pin = rospy.get_param('/lora_radio/m1_pin')
        aux_pin = rospy.get_param('/lora_radio/aux_pin')

        return Radio(serial_port, m0_pin, m1_pin, aux_pin)

    def transmit_callback(self,data):
        packets = Packet.multi_packetizer(data, last_packet)
        last_packet = packets[-1].packet_id
        for packet in packets:
            self.radio.transmit(packet)
    
    def gather_telemetry(self):
        telemetry = radio_telemetry()
        telemetry.last_packet_received = self.last_packet_received
        telemetry.missing_packets = tuple(self.missing_packets)
        self.telemetry_pub.publish(telemetry)
        self.missing_packets = []





if __name__ == "__main__":
    try:
        ComsNode()
    except rospy.ROSInterruptException:
        pass