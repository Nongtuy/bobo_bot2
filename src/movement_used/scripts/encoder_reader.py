import rospy as r
from std_msgs.msg import *


def callbackEncoderB(data);
    r.loginfo("encoder position_2", data.data)
    
def repeated_node();
    r.init_node("Left_Encoder", anonymous=True)
    r.Subscriber("left_ticks", Int16, callbackEncoderA)
    r.spin()

if __name__ == '__main__' :
    callbackEncoderB()

