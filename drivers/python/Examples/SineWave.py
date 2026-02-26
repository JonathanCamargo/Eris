#READ SineWave generated from teensy and publish as a ROS topic

import threading
import numpy as np


from threading import Thread,Lock
from time import sleep

from eris.eris import Eris

M=5     #Samples
dt=10.0/1000 #(s) sampling period of sineWave
TOTALTIME=10 #How long to run this node
RATE=10

#Setup eris
e=Eris(['SineWave'],['float'],[5],'/dev/ttyACM0')

def printData(data):
    #Print sinewave
    d=e.parse(data)
    allsamples=np.array([])
    for packet in d:
        samples=np.array(packet['SineWave'])
        n=np.isnan(samples)
	allsamples=np.concatenate([allsamples,samples[~n]])
    
    SineWave_dt=np.flip(np.arange(0,len(allsamples)),0)*dt*-1    
    for i,sample in enumerate(allsamples):
        print(sample)

print('Inicio')

for i in range(0,int(TOTALTIME*RATE)):
    #Read the data from eris
    out=e.read()
    #Every loop iteration we have m samples of every topic
    #And launch independent threads to publish
    if len(out)>0:
        #All in a thread
        t=Thread(target=printData,args=((out,)))
        t.start()
    sleep(1.0/RATE)

e.stop()

