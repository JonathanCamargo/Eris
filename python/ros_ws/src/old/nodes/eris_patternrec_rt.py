#!/usr/bin/env python
# Pattern recognition with Eris
# In this example:
# - Subscribe to features and do inference calling a matlab model

import rospy

import matlab.engine
from custom_msgs.msg import String,Float32,Float32MultiArray
from std_msgs.msg import Header,MultiArrayDimension

import numpy as np
import signal
import sys
import copy

eng=matlab.engine.start_matlab()


#TODO transform the modelpath into a rosparam
modelpath='/home/jim/Trials/jan15DELETE/testmodel.mat'
#load a model into the matlab engine
loadedData=eng.load(modelpath) #Load the model from a file
model=loadedData['m']
vecsize=int(eng.getfield(model,'inputSize'))
eng.predict(model,eng.zeros(1, vecsize)) #Preinitialize this so that everything loads before actually using the model

##################### ROS MESSAGES AND PUBLISHERS ##############################
classpub = rospy.Publisher('/record/eris/Class', String, queue_size=1)
featuresmsg=Float32MultiArray()
textmsg = String()

def publishClass(data):
    textmsg.data = data[0]
    textmsg.header=Header(stamp=rospy.Time.now())
    classpub.publish(textmsg)

def predict(msg):
    #Run matlab model and predict the class
    #featurespub.publish(featuresmsg)
    #print(msg.data)
    x=[v for v in msg.data]
    #print(x)
    x=matlab.double(x);
    #print(x)
    publishClass(eng.predict(model,x))
  
######################## HELPER FUNCTIONS ######################################
def signal_handler(sig,frame):
    ''' Terminate the node'''
    print('Ctrl+c')
    sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)

################################################################################

''' Main loop'''
rospy.init_node('imunode', anonymous=True)

feature_sub = rospy.Subscriber('record/eris/Features', Float32MultiArray, predict)

ROSRATE=50 #Hz
rate = rospy.Rate(ROSRATE)

print('Inicio')


while True:
    rate.sleep()

