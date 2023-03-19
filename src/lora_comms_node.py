#!/usr/bin/env python3
from drivers.radio import Radio, RadioResponseBad

from drivers.Depacketizer import Depacketizer
from drivers.Packet import Packet, multi_packetizer
import rospy
from std_msgs.msg import Bool, String
from roboat_pkg.msg import commanding, radio_telemetry



class ComsNode:
   
    def __init__(self):
        rospy.init_node('lora_comms')
        self.last_packet_sent = 0
        self.last_packet_received = 0
        self.missing_message = []
        self.radio = self.configure_radio()

        radio_in = rospy.Publisher('radio_in', commanding, queue_size=1)
        self.telemetry_pub = rospy.Publisher('telemetry', radio_telemetry, queue_size=1)
        rospy.Subscriber("radio_transmit", String, self.transmit_callback)

        poll_rate = rospy.get_param("lora_radio/poll_rate")
        rate = rospy.Rate(poll_rate)

        depacketizer = Depacketizer()
        rospy.loginfo("Radio Initialized, ready to listen")
        cycles = 0
        while not rospy.is_shutdown():
            depacketizer.write(self.radio.receive())
            messages = depacketizer.read_messages(self.missing_message_cb)
            for message in messages:
                radio_in.publish(message)
            cycles += 1
            if cycles % 15 == 0:
                self.gather_telemetry()
            rate.sleep()

    def missing_message_cb(self, missing_message):
        self.missing_message.append(missing_message)

    def configure_radio(self):
        serial_port = rospy.get_param('/lora_radio/serial_port')
        m0_pin = rospy.get_param('/lora_radio/m0_pin')
        m1_pin = rospy.get_param('/lora_radio/m1_pin')
        aux_pin = rospy.get_param('/lora_radio/aux_pin')

        return Radio(serial_port, m0_pin, m1_pin, aux_pin)

    def transmit_callback(self,data):
        data = data.value
        packets = multi_packetizer(bytes(data), self.last_packet_sent + 1)
        self.last_packet_sent = packets[-1].packet_id
        for packet in packets:
            self.radio.transmit(packet)
    
    def gather_telemetry(self):
        telemetry = radio_telemetry()
        telemetry.last_packet_rxd = self.last_packet_received
        telemetry.missing_packets = tuple(self.missing_packets)
        self.telemetry_pub.publish(telemetry)
        self.missing_packets = []





if __name__ == "__main__":
    try:
        ComsNode()
    except rospy.ROSInterruptException:
        pass