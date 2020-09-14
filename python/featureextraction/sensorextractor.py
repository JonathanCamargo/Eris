from featureextraction.subscriber import Subscriber
from featureextraction.featureextractor import FeatureExtractor
import numpy as np

class SensorExtractor:
    ''' A SensorExtractor object is a way to connect a FeatureExtractor with a signal from
    ROS.
    
    A sensor extractor defines an object to handle the window-based feature extraction for
    any message type from ROS. Create a FeatureExtractor first and configure it to produce
    the feature types you prefer. Then use that extractor to create an instance of the 
    SensorExtractor that subscribes to a topic of any type. At any time in your loop you
    can call the sensorextractor.extract() method which will return the feature values.
        
        '''

    def __init__(self,topicname,msgtype,extractor,window=250): #James, Jonathan (ros msgs)
        self.topicname=topicname
        self.msgtype=msgtype
        #Use the ros msgtype to determine the number of channels and their names

        self.window=window
        self.subscriber=Subscriber(topicname, msgtype, self.window+100)
        self.channelNames = self.subscriber.getChannels()
        self.excludeIndex = self.channelNames.index('header')

        #Create and configure the extractor based on channel names
        self.extractor=extractor
        self.extractor.configureHeader([v for i,v in enumerate(self.channelNames) if i != self.excludeIndex])
        self.header=self.extractor.header.tolist()
        self.subscriber.subscribe()


    def extract(self,extractor=None): 
        ''' Compute the features from this sensor. If you pass a FeatureExtractor it will
        use that extractor instead of the default assigned extractor created in construction
        '''
        if extractor==None:
            extractor=self.extractor
        # Extract the features for the current window      
        local=self.subscriber.getQueue()
        local=np.array(local)
        if local.size != 0:
            local=np.delete(local,self.excludeIndex,1)[-self.window:]
            return extractor.extract(local).tolist()
        else:
            return []

