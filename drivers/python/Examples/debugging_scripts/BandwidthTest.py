import rospy
import threading
import numpy as np
from std_msgs.msg import Header,String
from custom_msgs.msg import Float32

import copy

#ROS TEST to exceed WLAN bandwidth
from threading import Thread,Lock
from time import sleep

dataMutex=Lock()
rospy.init_node('talker', anonymous=True)

global publishers
publishers = []

RATE=50 #HZ
N=30  #Topics
M=5     #Samples
dt=1.0/(RATE*M)
TOTALTIME=10

for i in range(0,N):
    topicName='float'+str(i)
    publishers.append(rospy.Publisher(topicName, Float32, queue_size=10))

def publishDataNoThread(time,dt,data):
    global publishers
    for i,p in enumerate(publishers):
        for j,d in enumerate(data[i,:]):
            t=time-rospy.Duration(dt*(j-1))
            header=Header(stamp=t)
            msg=Float32(header,d)
            p.publish(msg)

def publishData(time,data):
    global publishers
    for i,p in enumerate(publishers):
        for j,d in enumerate(data[i,:]):
            t=time-rospy.Duration(dt*(j-1))
            header=Header(stamp=t)
            msg=Float32(header,d)
            publishers[i].publish(msg)

def wasteTime(time,data):
    sleep(1)

rate = rospy.Rate(RATE)

print('Inicio')

counter=0
t0=rospy.Time.now()
rate.sleep()

freeTime=[]
for i in range(0,int(TOTALTIME*RATE)):
    #Every loop iteration we have m samples of every topic
    #And launch independent threads to publish every topic

    #dataMutex.acquire(1)
    #data=np.random.rand(M,N)
    data=np.repeat([np.flip(np.arange(1,M+1),0)],[N],axis=0)
    data=data+counter

    counter=counter+M
    if counter+M>=1000:
        counter=0
    #dataMutex.release()
    time=rospy.Time.now()-t0
    time=rospy.Time()+time

    #One by one
    #for index,p in enumerate(publishers):
    #    d=data[index,:]
    #    t=Thread(target=publishData,args=(time,dt,p,d,))
    #    t.start()
    #All in a thread
    t=Thread(target=publishData,args=(time,data))
    #t=Thread(target=wasteTime,args=(time,data))
    t.start()
    #No threads
    #publishDataNoThread(time,dt,publishers,data)
    r=rate.remaining()
    freeTime.append(r.secs+r.nsecs/1E9)
    rate.sleep()

tend=rospy.Time.now()
dur=tend-t0
print('Runtime: '+str(dur.secs+dur.nsecs/1E9))
print('END')
print('FreeTime(ms)')
print('mean:'+str(np.mean(freeTime)*1000))
print('std:'+str(np.std(freeTime)*1000))
print('min:'+str(np.min(freeTime)*1000))
