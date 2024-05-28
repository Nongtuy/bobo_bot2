import rospy as r
from std_msgs.msg import *


def callbackEncoderA(data);
    r.loginfo("encoder position", data.data)
    
def repeated_node();
    r.init_node("Right_Encoder", anonymous=True)
    r.Subscriber("right_ticks", Int16, callbackEncoderA)
    r.spin()

if __name__ == '__main__' :
    callbackEncoderA()

