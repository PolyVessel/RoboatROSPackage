from drivers.radio import Radio, RadioResponseBad
import rospy

def configure_radio():
    serial_port = rospy.get_param('lora_radio/serial_port')
    m0_pin = rospy.get_param('lora_radio/m0_pin')
    m1_pin = rospy.get_param('lora_radio/m1_pin')
    aux_pin = rospy.get_param('lora_radio/aux_pin')

    return Radio(serial_port, m0_pin, m1_pin, aux_pin)

def test_radio(radio):
    try:
        radio.ping_radio()
    except RadioResponseBad as e:
        rospy.logfatal("Radio self-test failed! Reason: " + str(e))