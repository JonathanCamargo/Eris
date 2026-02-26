# Continuous feature extraction
from extractorhelper import *

import rospy
import signal
import sys

###################### Signal term ######################################
def signal_handler(sig,frame):
    print('Ctrl+c')
    sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)

################################## Loop ############################
e=ExtractorHelper(['example.yaml', 'example1.yaml'])

rospy.init_node('Featurenode', anonymous=True)

ROSRATE=200 #Hz
rate = rospy.Rate(ROSRATE)

e.subscribe()

while True:
    temp=e.features()
    if len(temp) != 0:
        print(temp)
    rate.sleep()


