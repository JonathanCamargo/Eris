#READ SineWave generated from teensy and publish as a ROS topic

import rospy
import threading
import numpy as np
from std_msgs.msg import Header,String
from custom_msgs.msg import Float32

from threading import Thread,Lock
from time import sleep

from eris.eris import Eris

rospy.init_node('talker', anonymous=True)

RATE=50 #HZ
M=5     #Samples
dt=10.0/1000 #(s) sampling period of sineWave
TOTALTIME=60 #How long to run this node

#Setup eris
e=Eris(['SineWave'],['float'],[5],'/dev/ttyACM0')

global publishers
publishers = []
#Create publishers
p_sineWave=rospy.Publisher('sinewave', Float32, queue_size=100)
publishers.append(p_sineWave)


def publishData(time,data):
    global publishers
    time=rospy.Time(time)
    #Publish sinewave
    d=e.parse(data)
    allsamples=np.array([])
    for packet in d:
        samples=np.array(packet['SineWave'])
        #print(samples)
        n=np.isnan(samples)
	allsamples=np.concatenate([allsamples,samples[~n]])
    
    SineWave_dt=np.flip(np.arange(0,len(allsamples)),0)*dt*-1    
    #print(SineWave_dt)
    for i,sample in enumerate(allsamples):
        time_i=time-rospy.Duration(SineWave_dt[i])
        header=Header(stamp=time_i)
        msg=Float32(header,sample)
        publishers[0].publish(msg)

rate = rospy.Rate(RATE)
print('Inicio')

t0=rospy.Time.now()
rate.sleep()

freeTime=[]
for i in range(0,int(TOTALTIME*RATE)):
    #Read the data from eris
    out=e.read()
    #Every loop iteration we have m samples of every topic
    #And launch independent threads to publish
    if len(out)>0:
        time=rospy.Time.now()-t0
        time=rospy.Time()+time
        #All in a thread
        t=Thread(target=publishData,args=(time.to_sec(),out))
        t.start()
    r=rate.remaining()
    freeTime.append(r.secs+r.nsecs/1E9)
    rate.sleep()
e.stop()
tend=rospy.Time.now()
dur=tend-t0
print('Runtime: '+str(dur.secs+dur.nsecs/1E9))
print('END')
print('FreeTime(ms)')
print('mean:'+str(np.mean(freeTime)*1000))
print('std:'+str(np.std(freeTime)*1000))
print('min:'+str(np.min(freeTime)*1000))
