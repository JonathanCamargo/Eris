# Feature extraction helper that configures feature extractors and organizes
from numpy_ringbuffer import RingBuffer
from featureextraction.subscriber import Subscriber
from featureextraction.featureextractor import FeatureExtractor
from featureextraction.sensorextractor import SensorExtractor

import numpy as np
import yaml

class ExtractorHelper:
    ''' Extractor helper is a class that facilitates the saving and configuration of multiple
    SensorExtractors with different configurations in yaml files. Please refer to SensorExtractor
    first before you use this advanced functionality.

    Create an ExtractorHelper using a list of configuration yaml files each yaml file contains
    a possible combination of features to be extracted.

    '''


    def __init__(self, configList):
        ''' Create an extractor helper from a list of configuration files.
         Each configuration file contains the definitions to set up subscribers, buffers    and selected features. Configuration "Blocks" are mapped to configIndex in the features() method.

         Construct the ExtractorHelper by passing a list of configuration yaml files.


         '''
        #Configuration is a list of dictionaries {'extractors':[],'sensorextractors':[],'windows':[],'indices':[]}
        self.configurations=[]
        self.sensorextractors=dict()

        if not(configList is list):
            configList=[configList]

        self._parseConfigurations(configList)
        print('Feature Helper ready!\nConfig indices:')


    def _parseConfigurations(self,configList):
        '''Read the configuration files and collect the total set of sensors to use'''

        configData=[]
        for configIdx,configFile in enumerate(configList):
            configData.append(self._checkConfig(configFile))

        #First sweep to get the topics and windows
        alltopics=[]
        allwindows=[]
        for configIdx,config in enumerate(configData):
            self.configurations.append({'extractors':[],'sensorextractors':[],'windows':[],'indices':[]})
            for item in config:
                if 'window' in item['opts'].keys():
                    window=item['opts']['window']
                else:
                    window=250 #Default window size
                allwindows.append(window)
                alltopics.append(item['topic'])

        alltopics_set=set(alltopics)
        maxwindow=dict()
        print(alltopics_set)
        for topic in alltopics_set:
            idx=[i for i,x in enumerate(alltopics) if x==topic]
            windows=[allwindows[i] for i in idx]
            maxwindow[topic]=max(windows)

        for configIdx,config in enumerate(configData):
            for item in config:
                e=FeatureExtractor(item['opts'])
                '''Configure the extractor or create a new SensorExtractor object
                if it is the first time of registration of this topic'''
                if item['topic'] in self.sensorextractors.keys():
                    se=self.sensorextractors[item['topic']]
                else:
                    if 'window' in item['opts'].keys():
                        window=item['opts']['window']
                        se=SensorExtractor(item['topic'],item['msgType'],window=maxwindow[item['topic']])
                    else:
                        se=SensorExtractor(item['topic'],item['msgType'],window=250)
                    self.sensorextractors[item['topic']]=se

                se.configureExtractor(e)
                # Use the header of the sensor extractor to determine the indices to use
                print(e.header)
                n=len(e.header.tolist())                
                indices=[e.header.tolist().index(v) for v in item['feats']]
                #Save the configuration for future use
                self.configurations[configIdx]['extractors'].append(e)
                self.configurations[configIdx]['sensorextractors'].append(se)
                self.configurations[configIdx]['windows'].append(window)
                self.configurations[configIdx]['indices'].append(indices)


    def features(self, configIndex=0):
        ''' Extract the features based on the current buffers' data'''
        features=[]
        config=self.configurations[configIndex]

        for i,se in enumerate(config['sensorextractors']):
            e=config['extractors'][i]
            w=config['windows'][i]
            indices=config['indices'][i]
            f=se.extract(e,w)
            if len(f)==0:
                return []
            features=features+[f[i] for i in indices]
        return features


    def _checkConfig(self, config):
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
        alldata=[]
        for key in keys:
            missing=([v for v in requiredKeys if v not in data[key]])
            if any(missing):
                error_msg='Invalid config! Required keys are missing...'
                for miss in missing:
                    error_msg=error_msg+' '+miss
                raise Exception(error_msg)
            alldata.append(data[key])
        return alldata


    def __str__(): # check how to?
        return "ExtractorHelper object:\nTopics:\t\t" + str(self.configuration)
