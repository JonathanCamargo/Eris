#!/usr/bin/env python
# READ EMG FEATURE DATA FROM ERIS AND SEND TO ROS TOPICS
# read EMG from ETI Arrays and offer the option to modify the reference selection

import rospy
import numpy as np
import signal
import sys

from custom_msgs.msg import Signal1CH,Signal8CH
from std_msgs.msg import Header,Float32MultiArray,MultiArrayDimension

from featureextraction import FeatureExtractor, SensorExtractor

##################### ROS MESSAGES AND PUBLISHERS ##############################
multiarraymsg=Float32MultiArray()
dim0=MultiArrayDimension()
dim0.label="feature"
dim0.size=9
dim0.stride=9
multiarraymsg.layout.dim.append(dim0)

features_publisher=rospy.Publisher('/features',Float32MultiArray,queue_size=1)
################################################################################


############################ ROS CALLBACKS #####################################


######################## HELPER FUNCTIONS ######################################
def signal_handler(sig,frame):
    ''' Terminate the connection to eris and close the node'''
    print('Ctrl+c')
    rosbag.stop()
    sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)

################################################################################
rospy.init_node('nextflexArrayFeatures', anonymous=True)

#Create a feature Extractor for time domain of emg features nextflexArrayFeatures
emg_feature_extractor=FeatureExtractor({'TD':True})
#Make a SensorExtractor that connects to the emg DATA
emg_sensor_extractor=SensorExtractor('/emg',Signal8CH,emg_feature_extractor)
#Create a feature Extractor for time domain of fsr features
fsr_feature_extractor=FeatureExtractor({'TD':True})
#Make a SensorExtractor that connects to the emg DATA
fsr_sensor_extractor=SensorExtractor('/fsr',Signal1CH,fsr_feature_extractor)

ROSRATE= 10 #Hz
rate = rospy.Rate(ROSRATE)
rate.sleep()

''' Main loop'''
while True:
    out=emg_sensor_extractor.extract()
    features_emg=out[::8]
    out=fsr_sensor_extractor.extract()
    features_fsr=out[0]
    
    multiarraymsg.data=np.concatenate([features_emg,np.expand_dims(features_fsr,0)])

    features_publisher.publish(multiarraymsg)

    #Wait a second
    rate.sleep()
