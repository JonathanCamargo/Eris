from subscriber import Subscriber #Import the Subscriber class
from time import sleep
import rospy
import signal
import sys

# Remember to do this somewhere in the node where we use anything related to ros
rospy.init_node('somenodename', anonymous=True)

def signal_handler(sig,frame):
    ''' Terminate the node gracefully'''
    print('Ctrl+c')
    sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)


#This class can create generic subscribers to our ros topics

somesubscriber=Subscriber('/sometopic','custom_msgs/String')

# The subscriber will be created and we can have acess to information from 
# the topic and msg type
print("topic:"+somesubscriber.topic)
print("Channels"+str(somesubscriber.channels))
print("Channel types"+str(somesubscriber.channel_types))

# This info can be just printed using print
print(somesubscriber)

# The subscriber has an internal buffer that can be specified with the argument
# queue_size... Can we just use this instead of the ring buffer? (James)

# To enable the callback just subscribe to the messages and then it will start 
# gathering data automatically. e.g.


print("SUBSCRIBING")
somesubscriber.subscribe()
print(somesubscriber)
for i in range(2*60): #Run for 2 minutes to test the callback
   q=somesubscriber.getQueue()
   print(q)
   sleep(1)
 

