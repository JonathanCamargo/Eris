# Feature extraction helper that configures feature extractors and organizes
from numpy_ringbuffer import RingBuffer
from featureextraction.subscriber import Subscriber
from featureextraction.featureextractor import *
from threading import Lock

import numpy as np
import yaml

class SensorExtractor:
    ''' Doc'''

    def __init__(self,topicname,msgtype,extractorOpts): #James, Jonathan (ros msgs)
        self.topicname=topicname
        self.msgtype=msgtype
        #Use the ros msgtype to determine the number of channels and their names

        if 'window' in extractorOpts:
            self.window=extractorOpts['window']
        else:
            self.window=250

        self.subscriber=Subscriber(topicname, msgtype, self.window+100)
        self.channelNames = self.subscriber.getChannels() 
        self.excludeIndex = self.channelNames.index('header') 
        
        #Create and configure the extractor based on channel names
        self.extractor=FeatureExtractor(extractorOpts)
        self.extractor.configureHeader([v for i,v in enumerate(self.channelNames) if i != self.excludeIndex])
        self.header=self.extractor.header.tolist()


    def extract(self, data_mutex): # James
        #may need to look at the deque->list->np.array transformation to see if it bottlenecks
        data_mutex.acquire(1)
        local=self.subscriber.getQueue()
        data_mutex.release()
            
        local=np.array(local)
        
        if local.size != 0:
            local=np.delete(local,self.excludeIndex,1)[-self.window:]
            return self.extractor.extract(local).tolist()
        else:   
            return []

class ExtractorHelper:
    ''' Doc '''


    def __init__(self, configList): 
        ''' Create an extractor helper from a list of configuration files.
         Each configuration file contains the definitions to set up subscribers, buffers    and selected features. Configuration "Blocks" are mapped to configIndex in the features() method.       
         '''
        self.data_mutex=Lock()
        self.sensors={}
        self.header=[]
        self.indices=[]
        self.topics=[]
        
        for i,config in enumerate(configList):
            # Parse file to get Topic, requested features, extractor opts, and message type
            data=self.parseConfig(config)
            self.indices.append([])

            # add sensor to end of extractor object list if it isn't in the list
            for topic in data:
                if topic not in self.sensors:
                    vals=data[topic]
                    self.sensors[topic]= SensorExtractor(vals['topic'],vals['msgType'],vals['opts'])
                    self.topics.append(topic)

                # generate global header, convert requested features to indices for this config
                self.header=self.header+self.sensors[topic].header
                self.indices[i]=self.indices[i]+[self.header.index(v) for v in vals['feats']]
  
        print('Feature Helper ready!\nConfig indices:')
        for i in range(0, len(self.indices)):
            print('\t\t' + str(i) + ':\t' + str(self.indices[i]))
        

    def subscribe(self):
        ''' Start the subscription to topics and create and enable callbacks '''
        for topic in self.topics:
            self.sensors[topic].subscriber.subscribe()


    def features(self, configIndex=0):
        ''' Extract the features based on the current buffers' data'''
        features=[]
        for topic in self.topics:
            features=features+self.sensors[topic].extract(self.data_mutex)
        if len(features) == len(self.header):
            return [features[i] for i in self.indices[configIndex]]
        return []


    def parseConfig(self, config):
        ''' config is a yaml file. Each topic is a dictionary containing fields 'feats',
        'opts', and 'msgType', 'topic'. feats is a list, opts is a dictionary, and msgtype is a single value. See  
        example.yaml. the topic is stored as the key for the entry
        ''' 
        with open(config, 'r') as f:
            data=yaml.safe_load(f)
        print(data)

        #Check each Topic and make sure we have the necessary fields
        requiredKeys=['feats', 'opts', 'msgType','topic']
        keys=data.keys()
        print(keys)
        for key in keys:
            missing=([v for v in requiredKeys if v not in data[key]])
            if any(missing):
                error_msg='Invalid config! Required keys are missing...' 
                for miss in missing:
                    error_msg=error_msg+' '+miss
                raise Exception(error_msg)
        return data


    def __str__(): # check how to?
        return "ExtractorHelper object:\nTopics:\t\t" + str(self.topics)
