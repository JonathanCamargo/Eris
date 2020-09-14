# Feature extraction helper that configures feature extractors and organizes
from numpy_ringbuffer import RingBuffer
from featureextraction.subscriber import Subscriber
from featureextraction.featureextractor import FeatureExtractor

import numpy as np
import yaml

class ExtractorHelper:
    ''' Extractor helper is a class that facilitates the saving and configuration of multiple
    SensorExtractors with different configurations in yaml files. Please refer to SensorExtractor
    first before you use this advance functionality. 
    
    Create an ExtractorHelper using a list of configuration yaml files each yaml file contains 
    a possible combination of features to be extracted.
    
    TODO THIS CLASS IS REALLY INNEFFICIENT AND OVERCOMPLICATED FOR NO REASON REWORK AS EITHER
    ONE Extractor per yaml file or a extractor with multiple yaml files but shared subscribers
    to minimize bandwidth.
    
    '''


    def __init__(self, configList):
        ''' Create an extractor helper from a list of configuration files.
         Each configuration file contains the definitions to set up subscribers, buffers    and selected features. Configuration "Blocks" are mapped to configIndex in the features() method.
         '''
        self.sensors={}
        self.header=[]
        self.indices=[]
        self.topics=[]

        if not(configList is list):
            configList=[configList]

        for i,config in enumerate(configList):
            # Parse file to get Topic, requested features, extractor opts, and message type
            data=self.parseConfig(config)
            self.indices.append([])

            # add sensor to end of extractor object list if it isn't in the list
            for topic in data:
                if topic not in self.sensors:
                    vals=data[topic]
                    if 'window' in vals['opts']:                        
                        self.sensors[topic]=SensorExtractor(
                            vals['topic'],vals['msgType'],vals['opts']['window'],vals['opts'])
                    else:
                        self.sensors[topic]=SensorExtractor(
                            vals['topic'],vals['msgType'],vals['opts'])
                    
                    self.topics.append(topic)

                # generate global header, convert requested features to indices for this config
                n=len(self.header)
                self.header=self.header+self.sensors[topic].header
                print(self.header)
                try:
                    self.indices[i]=self.indices[i]+[self.header.index(v,n) for v in vals['feats']]
                except Exception as e:
                    msger='\nPossible features are: {}'.format(self.header)
                    raise type(e)(str(e)+msger)


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
            features=features+self.sensors[topic].extract()
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
