#!/usr/bin/env python3
import rospy as R
# using Imt 32
from std_msgs.msg import *

# node name
Nodename = "Messagepublisher"
# Topic_Setting
TopicFrontleft ="Front_Left"
TopicFrontright ="Front_Right"
TopicBackleft ="Back_Left"
TopicBackright ="Bacl_Right"

# initailise node
R.init_node(Nodename,anonymous=True)

Pubfrontleft = R.Publisher(TopicFrontleft, Int32, queue_size=5)
Pubfrontright = R.Publisher(TopicFrontright, Int32, queue_size=5)
Pubbackleft = R.Publisher(TopicBackleft, Int32, queue_size=5)
Pubbackright = R.Publisher(TopicBackright, Int32, queue_size=5)

rate = R.Rate(1)


Front_Left = 0
Front_Right = 0
Back_Left = 0
Back_Right = 0

while not R.is_shutdown():
    R.loginfo(Front_Left)
    R.loginfo(Front_Right)
    R.loginfo(Back_Right)
    R.loginfo(Back_Right)
    Front_Left = int(input("Front-Left 0-255"))
    Front_Right = int(input("Front-Right 0-255"))
    Back_Left = int(input("Back-Left 0-255"))
    Back_Right = int(input("Back-Right 0-255"))
    Pubfrontleft.publish(Front_Left)
    Pubfrontright.publish(Front_Right)
    Pubbackleft.publish(Back_Left)
    Pubbackright.publish(Back_Right)
