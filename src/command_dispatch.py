import proto.coms_pb2 as protos
import rospy
from std_msgs.msg import String, Bool

publishers = {}

def init():
    rospy.init_node('command_dispatch', anonymous=True)
    rospy.Subscriber("radio_in", String, dispatch)
    publishers["e_stop"] = rospy.Publisher("e_stop", Bool)
    publishers["motor_set"] = rospy.Publisher("motor_set", String)

def dispatch(command_string):
    command_envelope = protos.Command.ParseFromString(command_string.data)
    handler = unknown_command
    match command_envelope.command:
        case protos.Command.E_STOP:
            handler = e_stop_handler
        case protos.Command.E_STOP_RELEASE:
            handler = e_stop_release_handler
        case protos.Command.MOTOR_SET:
            handler = motor_set_handler
        case protos.Command.MODE_SET:
            handler = mode_set_handler
    
    handler(command_envelope.command)
            
    
def e_stop_handler(command):
    publishers["estop"].publish(True)


def e_stop_release_handler(command):
    publishers["estop"].publish(False)

def motor_set_handler(command):
    pass

def mode_set_handler(command):
    pass

def unknown_command(command):
    pass

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass