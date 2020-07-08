#!/usr/bin/env python
# Recorder node receives commands from external nodes and launches rosbagrecord locally
# this is useful to have multiple hosts in the network without consolidating all the data 
# in a single computer (reduce latency).
#
# How to use?
#
# Run the rosrecorder node (rosrun recorder recorder.py)
# The node will be sitting watiing for a message in 
# 


import rospy
import rostopic
import signal
import sys

from roshandlers.rosbag import Rosbag
from custom_msgs.msg import String

from time import sleep
# Define: filename, topics to record,

defaultFilename='/home/ossip/test.bag'

#Record all the topics under the /record namespace


def signal_handler(sig,frame):
    print('Ctrl+c')
    sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)

activebag=None
def callback(msg):
    global activebag
    if ("record" in msg.data) and (activebag==None):
        parts=msg.data.split("record ")
        if (len(parts)>1):
            filename=parts[1]
        else:
            filename=defaultFilename
        topics=rospy.get_published_topics()
        topics=[info[0] for info in topics]
	#Do not record the relayed topics locally
	topics=[topic for topic in topics if "record" not in topic]
	#Also do not record rosout
	topics=[topic for topic in topics if "rosout" not in topic]  
        activebag=Rosbag(filename,topics)
        activebag.record()
    if ("stop" in msg.data) and (activebag!=None):
        activebag.stop()
        activebag=None



rospy.init_node('recorder', anonymous=True)

sub=rospy.Subscriber("/recorder/command",String,callback)

ROSRATE=1 #Hz
MAXRECORDINGSECONDS=10 #How long to record if no one terminates the recording
lastTime=rospy.Time.now()
while not rospy.is_shutdown():
    if activebag!=None:
        dt=rospy.Time.now()-lastTime
        if dt.to_sec()>MAXRECORDINGSECONDS:
            activebag.stop()
            activebag=None
    else:
        lastTime=rospy.Time.now()
