#!/usr/bin/env python
# Generic code for a classifier
#
# Subscribes to a feature vector (custom_msgs/Float32MultiArray) and a label (custom_msgs/String)
# Uses upcoming feature data to fit a classifier to predict the label

# Interface with topic command (Start/Stop learning)

import rospy
import numpy as np
import signal
import sys

import threading

import os
from EpicToolbox import FileManager,mkdirfile
from std_msgs.msg import String as StdString
from std_msgs.msg import Header
from custom_msgs.msg import String, Float32MultiArray

from datetime import date

# MODEL DEPENDENT CODE ? WRAP TO CLASS?
from sklearn.neural_network import MLPClassifier
from sklearn.metrics import accuracy_score
from sklearn.model_selection import KFold
from joblib import dump, load
from copy import deepcopy

from scipy import io

##################### ROS MESSAGES AND PUBLISHERS ##############################
stringmsg=String()
std_stringmsg=StdString()
labelpub = rospy.Publisher('prediction',String,queue_size=1)
logpub = rospy.Publisher('log', StdString, queue_size=50)
################################################################################
labels=[]
label=None
active_model=None
lock=threading.Lock()
learning = False

MAX_SAMPLES=100 #Number of samples per class to hold in memory
size=None #Size of the feature vector
memory=dict() #Sample data
numSamples=dict() #Number of samples
VERBOSE=2

# Setup a Rosbag
path=os.path.join(os.environ['HOME'],date.today().strftime('%m_%d_%y'))
mkdirfile(path)
f=FileManager(path,PathStructure=['Type','File'])
#rosparam=Rosparam('/')

############################ ROS CALLBACKS #####################################
def learning_callback(msg):
    '''Enable or disable learning'''
    global learning
    if msg.data=='START':
        printAndLog('Learning enabled')
        learning=True
    elif msg.data=='STOP':
        printAndLog('Learning disabled')
        learning=False

def label_callback(msg):
    global labels,label,size,memory,numSamples,active_model
    print('Label:{}'.format(msg.data))
    lock.acquire()
    label=msg.data
    if label in labels:
        pass
    else:
        print('\t New label to the classifier')
        if size==None:
            lock.release()
            return
        labels.append(label)
        memory[label]=np.zeros((MAX_SAMPLES,size))
        numSamples[label]=0
        active_model=None #Reset the model since the number of labels changed
    lock.release()

def labelstd_callback(msg):
    stringmsg.header.stamp=rospy.Time.now()
    stringmsg.data=msg.data
    label_callback(stringmsg)

def features_callback(msg):
    ''' Get a new feature sample and incorporate the sample in memory'''
    global active_model,labels,label,memory,numSamples,size,learning

    if learning == False:
        size=msg.layout.dim[0].size

    if learning == True:
        lock.acquire()
        if label==None:
            size=msg.layout.dim[0].size
            lock.release()
            return

        # Add the sample to the buffers for the corresponding label
        x=memory[label]
        idx=numSamples[label]
        if idx<MAX_SAMPLES:
            x[idx,:]=msg.data
            numSamples[label]=numSamples[label]+1
        else:
            x=np.roll(x,1,axis=0)
            x[0,:]=msg.data
            memory[label]=x
            numSamples[label]=numSamples[label]+1
        lock.release()

    # Compute the prediction from the active model
    if active_model==None:
        return

    lock.acquire()
    out=active_model.predict(np.array([msg.data]))
    lock.release()
    stringmsg.header.stamp=rospy.Time.now()
    stringmsg.data=out[0]
    labelpub.publish(stringmsg)
    #publish output



######################## HELPER FUNCTIONS ######################################
def signal_handler(sig,frame):
    ''' Terminate the connection to eris and close the node'''
    print('Ctrl+c')
    rosbag.stop()
    sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)

def printAndLog(strdata):
    ''' Print and publish string data to the log '''
    print(strdata)
    std_stringmsg.data=strdata
    logpub.publish(std_stringmsg)

def memory2xy(memory):
    '''Convert the data from memory to a x,y tables for fitting a model'''
    labels=memory.keys()
    x=[]
    y=[]
    for l in labels:
        x.append(memory[l])
        y.append([l]*memory[l].shape[0])
    x=np.concatenate(x)
    y=np.concatenate(y)
    return x,y

def retrain(memory):
    global active_model
    mdl = deepcopy(active_model)
    x,y=memory2xy(memory)
    mdl.partial_fit(x,y)
    return mdl

def train(memory):
    lr=0.05
    tol=0.001
    mdl=MLPClassifier(hidden_layer_sizes=(10,10),max_iter=300,learning_rate_init=lr,tol=tol,verbose=VERBOSE)
    x,y=memory2xy(memory)
    mdl.fit(x,y)
    return mdl

def CheckPoint():
    global active_model,memory
    '''Save a copy of the current model and the data in memory'''
    modelfiles=f.fileList({'Type':'model','File':'*.mdl'})
    nextmodel=f.genList({'Type':'model','File':'{:01d}.mdl'.format(len(modelfiles)+1)})
    nextmemory=f.modFileList(nextmodel,{'Type':'memory','ext':'mat'})
    #Save the model

    printAndLog('Model checkpoint ({})'.format(nextmodel[0]))
    print(nextmemory)
    mkdirfile(nextmemory[0])
    mkdirfile(nextmodel[0])
    dump(active_model,nextmodel[0])
    io.savemat(nextmemory[0],{'data':memory})

################################################################################
''' Main loop'''
rospy.init_node('classifier', anonymous=True)
labelsub = rospy.Subscriber('label',String,label_callback)
labelstdsub = rospy.Subscriber('labelstd',StdString,labelstd_callback)
learningsub = rospy.Subscriber('train',String,learning_callback)
learningstdsub = rospy.Subscriber('train_std',StdString,learning_callback)
featuressub = rospy.Subscriber('features',Float32MultiArray,features_callback)

ROSRATE= 1 #Hz
rate = rospy.Rate(ROSRATE)
rate.sleep()

elapsed=0
lasttime=rospy.Time.now()

# Restore from previous model
models=f.fileList({'Type':'model','File':'*.mdl'})

if len(models)>0:
   print('Previous models found:\n\t{}'.format(models))
   memoryfiles=f.modFileList(models,{'Type':'memory','ext':'mat'})
   active_model=load(models[-1])
   memory=io.loadmat(memoryfiles[-1])
   print(memory.keys())


count=0
while True:
    time=rospy.Time.now()
    if ((not label==None) and (label in numSamples.keys())):
        printAndLog('label={}({}samples)'.format(label,numSamples[label]))

    if labels==[]:
        rate.sleep()
        continue

    n=[numSamples[l] for l in labels]
    print(n)
    if (learning and (count % 5 == 0)):
        if (active_model==None) and (np.all(np.greater(n,MAX_SAMPLES))):
            mdl=train(memory)
            lock.acquire()
            active_model=deepcopy(mdl)
            lock.release()
            CheckPoint()
        elif active_model!=None:
            mdl=retrain(memory)
            lock.acquire()
            active_model=mdl
            lock.release()
            CheckPoint()

    count=count+1

    #Wait a second
    rate.sleep()
