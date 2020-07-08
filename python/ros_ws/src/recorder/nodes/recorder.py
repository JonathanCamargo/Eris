#!/usr/bin/env python
# Recorder node receives commands from external nodes and launches rosbagrecord only on
# specific topics. 
# This is useful to have multiple hosts in the network without consolidating all the data 
# in a single computer (reduce latency).
#
# How to use?
#
# Run the rosrecorder node (rosrun recorder recorder.py name:=<somename> )
#
# This node will subscribe to <somename>/command listening to commands:
#           start <localfilepath.bag> : start recording
#           stop  : stop recording
#
# The node records all the topics that are explicitely stated under the rosparam /<somename>/topics
# 


import rospy
import rostopic
import signal
import sys

from roshandlers.rosbag import Rosbag
from custom_msgs.msg import String

from time import sleep

# Define: default filename if start command does not include it.  Topics to record.
defaultFilename='/home/ossip/test.bag'
topicsList=[]


def signal_handler(sig,frame):
    print('Ctrl+c')
    sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)

activebag=None
def callback(msg):
    ''' Callback to process commands
    Commands can be: start <filepath.bag>  # Starts recording the predefined topics
                     stop                  # Stop recording the current bag file 
    '''
    global activebag
    global topicsList
    if ("start" in msg.data) and (activebag==None):
        parts=msg.data.split("start ")
        if (len(parts)>1):
            filename=parts[1]
        else:
            filename=defaultFilename
        topics=rospy.get_published_topics()
        topics=[info[0] for info in topics]
	#Do not record the relayed topics locally
	topics=[topic for topic in topics if topic in topicsList]
        activebag=Rosbag(filename,topics)
        activebag.record()
    if ("stop" in msg.data) and (activebag!=None):
        activebag.stop()
        activebag=None



rospy.init_node('recorder', anonymous=True)
rospy.getparam(
sub=rospy.Subscriber("/recorder/command",String,callback)

ROSRATE=1 #Hz
MAXRECORDINGSECONDS=600 #How long to record if no one terminates the recording
lastTime=rospy.Time.now()
while not rospy.is_shutdown():
    if activebag!=None:
        dt=rospy.Time.now()-lastTime
        if dt.to_sec()>MAXRECORDINGSECONDS:
            activebag.stop()
            activebag=None
    else:
        lastTime=rospy.Time.now()
