import proto.coms_pb2 as protos
import rospy
from std_msgs.msg import String

def init():
    rospy.init_node('command_dispatch', anonymous=True)
    rospy.Subscriber("command_dispatch", String, dispatch)

def dispatch(command_string):
    command = protos.Command.ParseFromString(command_string.data)
    match command.command:
        case protos.Command.E_STOP:
            print("E-Stop")
        case protos.Command.E_STOP_RELEASE:
            print("E-Stop Release")
        case protos.Command.MOTOR_SET:
            print("Thrust")
        case protos.Command.MODE_SET:
            print("Mode Set")
        
    


        
if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass