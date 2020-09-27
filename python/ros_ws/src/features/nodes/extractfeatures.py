#!/usr/bin/env python
# Feature extraction and publishing
import rospy
import rostopic
import signal
import sys
import rospkg

from rospy import Publisher
from std_msgs.msg import MultiArrayDimension,Float32
from custom_msgs.msg import Float32MultiArray

from featureextraction.extractorhelper import *


# Publishers and subscribers
featurespub=Publisher('/features',Float32MultiArray,queue_size=1)
feat0pub=Publisher('/feature0',Float32,queue_size=1)
feat1pub=Publisher('/feature1',Float32,queue_size=1)

# Messages
featuresmsg=Float32MultiArray()

############################ Signal term ######################################
def signal_handler(sig,frame):
    print('Ctrl+c')
    sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)

###################### Publishing and callback functions ###################
def publishFeatures(sample):
    featuresmsg.data=sample
    featuresmsg.layout.dim=[MultiArrayDimension()]
    featuresmsg.layout.dim[0].size=len(sample)
    featuresmsg.layout.dim[0].stride=1
    featuresmsg.layout.dim[0].label='index'
    featurespub.publish(featuresmsg)

################################## Loop ############################

rospy.init_node('Featurenode', anonymous=True)
print("helloooo")
print(rospy.get_param('~featuresfile'))
extractor=ExtractorHelper(rospy.get_param('~featuresfile'))


ROSRATE=33 #Hz
rate = rospy.Rate(ROSRATE)


msg=Float32()
while True:
    feats=extractor.features()
    if len(feats) != 0:
        #print(feats)
        publishFeatures(feats)
        msg.data=feats[0]
        feat0pub.publish(msg)
        msg.data=feats[1]
        feat1pub.publish(msg)


    rate.sleep()
