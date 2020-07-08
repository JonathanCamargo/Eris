#!/usr/bin/env python
# Relay node takes a list of topics and republish prepending /record namespace

import rospy
import rostopic
import signal
import sys


QUEUE_SIZE=1000 #Make sure we don't miss points

def signal_handler(sig,frame):
    print('Ctrl+c')
    sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)

def echo(pub,msg):
    ''' echos the message to a publisher '''
    pub.publish(msg)


rospy.init_node('talker', anonymous=True)

# Get the list of topics to relay from rosparam

publishers=[]
subscribers=[]

# Manually list the topics to Relay
topics=['/emg']

for topic in topics:
    #relay
    (topicClass,topicName,c)=rostopic.get_topic_class(topic,blocking=True)
    print("Relay for "+topicName+" with class "+str(topicClass))
    pub = rospy.Publisher("/record"+topicName, topicClass, queue_size=QUEUE_SIZE)
    callback=lambda msg: echo(pub,msg)
    sub = rospy.Subscriber(topic, topicClass,callback)

    publishers.append(pub)
    subscribers.append(sub)

rospy.spin()
