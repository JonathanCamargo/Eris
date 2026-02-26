# Continuous feature extraction
from featureextraction.extractorhelper import *
from custom_msgs.msg import Float32MultiArray
from std_msgs.msg import Header, Bool, MultiArrayDimension

import rospy
import signal
import sys

###################### Signal term ######################################
def signal_handler(sig,frame):
    print('Ctrl+c')
    sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)



##################### ROS MESSAGES AND PUBLISHERS ##############################
featuresmsg=Float32MultiArray()

featpub = rospy.Publisher('/eris/Features', Float32MultiArray, queue_size=3)

##################### HELPER FUNCTIONS #######################################
def publishFeaturesHelper(sample, publisher):
    '''Publish features data'''
    timestamp=rospy.Time.now()
    featuresmsg.data=sample
    featuresmsg.header=Header(stamp=timestamp)
    featuresmsg.layout.dim=[MultiArrayDimension()]
    featuresmsg.layout.dim[0].size=len(sample)
    featuresmsg.layout.dim[0].stride=1
    featuresmsg.layout.dim[0].label='index'
    publisher.publish(featuresmsg)

################################## Loop ############################
e=ExtractorHelper(['example.yaml'])

rospy.init_node('Featurenode', anonymous=True)

ROSRATE=20 #Hz
rate = rospy.Rate(ROSRATE)

e.subscribe()

while True:
    temp=e.features() #Just publish features at the rate defined here
    if len(temp) != 0:
        publishFeaturesHelper(temp,featpub)
    rate.sleep()


