#!/usr/bin/env python
# Publish Features from multiple sensors in a single feature vector

import rospy
import threading

from custom_msgs.msg import EMG8CH, IMU, Float32, String, featmsg, Float32MultiArray
from std_msgs.msg import Header, Bool, MultiArrayDimension

from eris.eris import Eris
from threading import Thread,Lock

#from time import sleep
import numpy as np
from numpy_ringbuffer import RingBuffer
from featureextractor import *
import struct
from construct import Array,Struct,Float32l,Int8ub,this
import signal
import sys
import copy

#import Exception

dataMutex=Lock()

#If requested_feats is set to something besides None, the program won't parse the file 

#If CHANNEL_NAMES is set to something besides None, the program won't generate default names 
#for channels within a topic (Ch0 for emg, Accel_X for imu, etc)
requested_feats = {'imu': ['Accbuff[sensor][0]el_X_mean', 'Accel_X_std', 'Accel_Z_max']}
CHANNEL_NAMES = None
############################Get Feature Mask##############################
#MATLAB code to generate feature mask files:
#xopts is the options struct used when creating a model
#TODO: change this so that channel name is everything but the lowest level, eg imu_thigh
        #fileID = fopen('testfile.txt', 'w'); 
        #fields = fieldnames(xopts); 
        #for i = 1:length(fields)
        #    field = fields{i};
        #    fprintf(fileID, [field '\n']); 
        #    features = xopts.(field); 
        #    for j = 1:length(features)
        #        fprintf(fileID, ['\t' features{j} '\n']);
        #   end
        #end
        #fclose(fileID);

#creates a dictionary with features seperated by sensor type
#    INPUT:         |   OUTPUT:     
#emg       |requested_fields = {'emg':['Ch0_rms','Ch0_zeroCross','Ch1_rms']}
#	Ch0_rms         |
#	Ch0_zeroCross   |
#	Ch1_rms         |
if requested_feats == None:
    maskFile = '/home/jim/Trials/jan15DELETE/testfile.txt' 
    with open(maskFile, 'r') as f: 
        fieldName = f.readline().rstrip()
        requested_feats = {fieldName:[]}
        for line in f:
            if line[0] == '\t':
                requested_feats[fieldName].append(line[1:].rstrip())  
            else:
                fieldName = line.rstrip()
                requested_feats[fieldName] = []

######################Feature Extractor Setup#############################
#this is the number of "channels" for a single sensor of a each type
NUM_CHANNELS = {'EMG':8, 'FSR':1, 'IMU':6}
if CHANNEL_NAMES == None:
    CHANNEL_NAMES = {}
    CHANNEL_NAMES['FSR'] = ['data']; 
    CHANNEL_NAMES['EMG'] = ['Ch'+str(i) for i in range(0, NUM_CHANNELS['EMG'])]
    CHANNEL_NAMES['IMU'] = ['Accel_X', 'Accel_Y', 'Accel_Z', 'GYRO_X', 'Gyro_Y', 'Gyro_Z']
WINDOW_SIZE = 63
INC_SIZE = 13
numSamples = 0
buff = {}
extractors = {}
#Right now, this is only the EMG set of buffers, extractors, and feature indices

sensors = [k for k in requested_feats.keys()]; 
header = []
indices = []
for sensor in sensors:
    sensorType = sensor[0:3].upper()
    buff[sensor] = []
    extractors[sensor] = [] 
    for i in range(0, NUM_CHANNELS[sensorType]):
        buff[sensor].append(RingBuffer(capacity=400))
    extractors[sensor] = FeatureExtractor({'window': WINDOW_SIZE, 'slide': INC_SIZE, 'TD': True})
    extractors[sensor].channels = NUM_CHANNELS[sensorType]
    extractors[sensor].configureHeader(CHANNEL_NAMES[sensorType])

    header = header + [v.lower() for v in extractors[sensor].header.tolist()]
    indices = indices + [header.index(v.lower()) for v in requested_feats[sensor]]

print("Channels:\t\t" + str(NUM_CHANNELS))
print("Requested Features:\t" + str(requested_feats))
print("Feature Indices:\t" + str(indices))

##################### ROS MESSAGES AND PUBLISHERS ########################
featpub = rospy.Publisher('/eris/Features', Float32MultiArray, queue_size=3)
featuresmsg = Float32MultiArray()
msgs = {'EMG': EMG8CH, 'FSR': Float32, 'IMU': IMU}

######################## HELPER FUNCTIONS ################################
def signal_handler(sig,frame):
    print('Ctrl+c')
    sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)

######################## PUBLISHER FUNCTIONS #############################
def publishFeatures(data, publisher):
    featuresmsg.header = Header(stamp=rospy.Time.now())
    featuresmsg.data = np.array(data)[indices]
    #featuresmsg.layout.dim=[MultiArrayDimension(), MultiArrayDimension()]
    #featuresmsg.layout.dim[0].size=NUM_CHANNELS
    #featuresmsg.layout.dim[0].stride=len(data)
    #featuresmsg.layout.dim[0].label='Channel'
    #featuresmsg.layout.dim[1].size=len(data) / NUM_CHANNELS
    #featuresmsg.layout.dim[1].stride=len(data) / NUM_CHANNELS
    #featuresmsg.layout.dim[1].label='Feature'
    featpub.publish(featuresmsg)    

######################## CALLBACK FUNCTIONS ##############################
def callback_helper(msg, sensor):
    global numSamples
    sensorType = sensor[0:3].upper()
    if sensorType == 'EMG':
        buff[sensor][0].append(msg.ch0)
        buff[sensor][1].append(msg.ch1)
        buff[sensor][2].append(msg.ch2)
        buff[sensor][3].append(msg.ch3)
        buff[sensor][4].append(msg.ch4)
        buff[sensor][5].append(msg.ch5)
        buff[sensor][6].append(msg.ch6)
        buff[sensor][7].append(msg.ch7)

    elif sensorType == 'IMU':
        buff[sensor][0].append(msg.ax)
        buff[sensor][1].append(msg.ay)
        buff[sensor][2].append(msg.az)
        buff[sensor][3].append(msg.wx)
        buff[sensor][4].append(msg.wy)
        buff[sensor][5].append(msg.wz)
        numSamples = numSamples + 1; 
    
    elif sensorType == 'FSR':
        buff[sensor][0].append(msg.data)
    
########################### Program Main body ############################
rospy.init_node('Featurenode', anonymous=True)

#registering all subscribers - this seems to work
subscribers = {}
for sensor in sensors:
    sensorType = sensor[0:3].upper()
    topicName = '/eris/' + sensor
    callback = lambda msg, s=sensor: callback_helper(msg, s) 
    subscribers[sensor] = rospy.Subscriber(topicName, msgs[sensorType], callback)

ROSRATE=200 #Hz
rate = rospy.Rate(ROSRATE)

while True:
    dataMutex.acquire(1)
    if numSamples >= WINDOW_SIZE:
        #should copy all data before beginning to extract?
        local = {} 
        for sensor in sensors: 
            sensorType = sensor[0:3].upper()
            local[sensor] = np.array(buff[sensor][0])[-WINDOW_SIZE:]
            for i in range(1, NUM_CHANNELS[sensorType]):
                local[sensor] = np.vstack((local[sensor], np.array(buff[sensor][i])[-WINDOW_SIZE:])) 
        dataMutex.release()
        
        local[sensors[0]] = np.transpose(local[sensors[0]])
        features = extractors[sensors[0]].extract(local[sensors[0]]).tolist()
        for sensor in sensors[1:]:
            local[sensor] = np.transpose(local[sensor])
            features = features + extractors[sensor].extract(local[sensor]).tolist()

        publishFeatures(features, featpub)
        numSamples = numSamples - INC_SIZE
        del local
    else: 
        dataMutex.release()
        
    rate.sleep()
    
