#!/usr/bin/env python
# Feature extraction and publishing
import rospy
import rostopic
import signal
import sys
import rospkg

from rospy import Publisher
from std_msgs.msg import MultiArrayDimension,Float32
from std_msgs.msg import Float32MultiArray as Float32MultiArrayStd
from custom_msgs.msg import Float32MultiArray

from featureextraction.extractorhelper import *


# Publishers and subscribers
featurespub=Publisher('/features',Float32MultiArray,queue_size=1)
featuresstdpub=Publisher('/featuresstd',Float32MultiArrayStd,queue_size=1)

# Messages
featuresmsg=Float32MultiArray()
featuresmsgstd=Float32MultiArrayStd()

############################ Signal term ######################################
def signal_handler(sig,frame):
    print('Ctrl+c')
    sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)

###################### Publishing and callback functions ###################
def publishFeatures(sample):
    featuresmsg.header.stamp=rospy.Time.now()
    featuresmsg.data=sample
    featuresmsg.layout.dim=[MultiArrayDimension()]
    featuresmsg.layout.dim[0].size=len(sample)
    featuresmsg.layout.dim[0].stride=1
    featuresmsg.layout.dim[0].label='index'
    featurespub.publish(featuresmsg)

def publishFeaturesStd(sample):
    featuresmsgstd.data=sample
    featuresmsgstd.layout.dim=[MultiArrayDimension()]
    featuresmsgstd.layout.dim[0].size=len(sample)
    featuresmsgstd.layout.dim[0].stride=1
    featuresmsgstd.layout.dim[0].label='index'
    featuresstdpub.publish(featuresmsgstd)

################################## Loop ############################

rospy.init_node('Featurenode', anonymous=True)

print("Starting FeatureNode")

if not (rospy.has_param('~featuresfile')):
    rospy.logerr("Features file argument missing")
    signal_handler(None,None)

extractor=ExtractorHelper(rospy.get_param('~featuresfile'))

ROSRATE=33 #Hz
rate = rospy.Rate(ROSRATE)


msg=Float32()
while True:
    feats=extractor.features()
    if len(feats) != 0:
        #print(feats)
        publishFeatures(feats)
        publishFeaturesStd(feats)

    rate.sleep()
