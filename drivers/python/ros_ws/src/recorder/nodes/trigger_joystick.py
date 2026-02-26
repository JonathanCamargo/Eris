#!/usr/bin/env python
# Use data from a joystick to trigger the recording start/stop events.
# How to use?
#
# Set the param recorder/filename to the filepath that you wish to save the file
#  (do not .bag extension as the trigger with add a suffix _xx.bag to the filename)
#
# This node will subscribe to cmd_vel listening to the speed of a joystick
#           if linear.x > 0.5 sends start command (incrementing filename by 1)
#           if linear.x < -0.5  sends stop command.
#
import rospy
import rostopic
import signal
import sys

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from time import sleep

# Define: default filename if start command does not include it.  Topics to record.
defaultFilename='/home/ossip/scratch/test'
filename=defaultFilename
direction=None
fileCounts=0
isRecording=False

def signal_handler(sig,frame):
    print('Ctrl+c')
    sys.exit(0)

def NextRecording(pub,filename):
    #Send a command to the recorder to initiate a new recording
    msg=String()
    msg.data="start "+ filename + '_' + str(fileCounts) +'.bag'
    pub.publish(msg)

def StopRecording(pub):
    #Send a command to the recorder to initiate a new recording
    msg=String()
    msg.data="stop"
    pub.publish(msg)

def callback(msg):
    global direction
    global fileCounts
    global isRecording
    if msg.linear.x>0.5 and direction!='up':
        direction='up'
        if not(isRecording):
            fileCounts=fileCounts+1
            NextRecording(pub,filename)
            isRecording=True
            print('Start recording')
    elif msg.linear.x<-0.5 and direction!='down':
        direction='down'
        if isRecording:
            StopRecording(pub)
            isRecording=False
            print('Stop recording')
    elif (msg.linear.x<0.5) and (msg.linear.x>-0.5):
        direction=None

signal.signal(signal.SIGINT,signal_handler)
rospy.init_node('trigger_joystick', anonymous=True)
sub=rospy.Subscriber("/cmd_vel",Twist,callback)
pub=rospy.Publisher("/recorder/command",String,queue_size=1)

if rospy.has_param("recorder/filename"):
    filename=rospy.get_param("recorder/filename")

ROSRATE=0.1 #Hz
r=rospy.Rate(ROSRATE)
while not rospy.is_shutdown():
    r.sleep()
