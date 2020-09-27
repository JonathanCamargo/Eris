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

    def __init__(self,topicname,msgtype,extractor=None,window=250):
        self.topicname=topicname
        self.msgtype=msgtype
        #Use the ros msgtype to determine the number of channels and their names

        self.window=window
        self.subscriber=Subscriber(topicname, msgtype, self.window+100)
        self.channelNames = self.subscriber.getChannels()
        self.excludeIndex = self.channelNames.index('header')

        #Create and configure the extractor based on channel names
        if extractor!=None:
            self.extractor=extractor
            self.configureExtractor(self.extractor)
            self.header=self.extractor.header.tolist()

        self.subscriber.subscribe()

    def configureExtractor(self,extractor):
        '''Configure a extractor header for this sensor's channels'''
        extractor.configureHeader([v for i,v in enumerate(self.channelNames) if i != self.excludeIndex])


    def extract(self):
        ''' Compute the features from this sensor using the extractor and window for this instance and clear the window.'''
        return self.extract(self.extractor,self.window)


    def extract(self,extractor,window):
        ''' Compute the features from this sensor using a extractor and a window.'''
        if window<self.window:
            print("Window is bigger than data buffers")
            raise
        local=self.subscriber.getQueue()
        local=np.array(local)
        if local.size != 0:
            local=np.delete(local,self.excludeIndex,1)[-self.window:]
            return extractor.extract(local).tolist()
        else:
            return []
