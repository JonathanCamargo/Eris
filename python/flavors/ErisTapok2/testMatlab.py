''' Basic example of reading data from ErisTapok2'''
'''
from eris.eris import Eris

#Use construct to create a good c format
from construct import Array,Struct,Float32l,Int8ub,this
from time import sleep
NUMFSRCHANNELS=1

FSRSample_t = Struct(
    "timestamp" / Float32l,
    "ch" / Float32l[NUMFSRCHANNELS]
    )
    
format=Struct(
    "len" / Int8ub,
    "fsrdata" / Array(this.len,FSRSample_t)
)

e=Eris(['FSR'],[format],'COM3')

sleep(1) 
e.start()
for i in range(0,1000):
    z=e.read() #Read all pending packets
    ndpackets=len(z['D']) #Check D packets
    if (ndpackets>0):
        for packet in z['D']:
            data=e.parse(packet)
            print(data)
    sleep(1.0/100.0)
e.stop()

'''

import matlab.engine
import time
import random
eng=matlab.engine.start_matlab()

loadedData=eng.load("D:\Dropbox (GaTech)\modeltest\model.mat") #Load the model from a file

model=loadedData['z']

for i in range(0,100):
    eng.predict(model,0) #Preinitialize this

xtest=[random.randint(0,10)+random.random() for i in range(0,1000)]

start=time.time()
for x in xtest:    
    y=eng.predict(model,x)
    #print(y)

end=time.time()

print('Elapsed time:'+str(end-start))