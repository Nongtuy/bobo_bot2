#!/usr/bin/env python3
import rospy as R
# using Imt 32
from std_msgs.msg import *
def EncoderFLcallback(EnFL):
    R.loginfo("Fron Left Encoder at", EnFL.data)

def EncoderFRcallback(EnFR):    
    R.loginfo("Fron Right Encoder at", EnFR.data)
    
def EncoderBLcallback(EnBL):
    R.loginfo("Fron Left Encoder at", EnBL.data)
    
def EncoderBRcallback(EnBR):
    R.loginfo("Fron Left Encoder at", EnBR.data)
def Encoders_Reader():
    R.init_node("Encoder Reader", anonymous=True)
    R.Subscriber("Encoder_FL", Int32, EncoderFLcallback)
    R.Subscriber("Encoder_FR", Int32, EncoderFRcallback)
    R.Subscriber("Encoder_BL", Int32, EncoderBLcallback)
    R.Subscriber("Encoder_BR", Int32, EncoderBRcallback)
    R.spin()
if __name__=="__main__":
    Encoders_Reader()

