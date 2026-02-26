import numpy as np
from scipy import stats, linalg
# import pywt
# from util import *

DEBUG = False
class FeatureExtractor:

    ''' Instructions for understanding this class:

    # A class to extract feature vectors from data
    #
    # Usage:
    # 1. Set up a FeatureExtractor using the constructor with the constructor
    # extractor=FeatureExtractor(name_value_pair_options);
    # 2. FEATURE_TABLE=extractor.extract(X) where X is a vector (or matrix) with columns
    # containing a window of data with each row corresponding to a sample
    # inside the window.
    #
    # Name value pair options are: (defaults)
    # - 'TD' true/(false)     Time domain features
    # - 'STAT' true/(false)   Simple statistical features: mean, min, max, std
    # - 'AR' true/(false)     Auto recursive
    # - 'AROrder'	(4)       Auto recursive order
    # - 'EN'  true/(false)    Entropy
    # - 'WT'   true/(false)   Wavelet transform // DISABLE
    # - 'wname' ('coif1')     Mother wavelet
    # - 'mean' true/(false)   Mean over window
    '''

    # Instead of tables, use dictionaries?
        # Number of labels becomes the number of keys
        # field names becomes the array of keys
    header = np.array([])

    # featureOptions is a dictionary
    featureOptions = {};

    isReady = False
    channels = 0
    features_header = np.array([])


    ################### Static Methods ########################################
    # Actual features that we can extract
    # All of these should return arrays

    # Time domain based feature extraction
    @staticmethod
    def TD_extract(self, data_window):  # Completed
        rms = np.sqrt(np.mean(data_window**2))
        mean = self.MEAN_extract(self,data_window)

        zeroCross = np.sum(np.absolute(np.diff(data_window>0)))
        slopeSignChange = np.sum(np.absolute(np.diff(np.diff(data_window)>0)))
        waveformLength = np.sum(np.absolute(np.diff(data_window)))

        minVal = np.amin(data_window)
        maxVal = np.amax(data_window)
        stdVal = np.std(data_window)

        return np.array([rms,mean,zeroCross,slopeSignChange,waveformLength,minVal, maxVal, stdVal])

    # Simple statisical based feature extraction
    @staticmethod
    def STAT_extract(self, window):  # Completed

        mean = self.MEAN_extract(self,window)
        minVal = np.amin(window)
        maxVal = np.amax(window)
        stdVal = np.std(window)
        # startVal = window[0]
        endVal = window[-1]

        return np.array([mean, minVal, maxVal, stdVal, endVal])
        # return np.array([mean, minVal, maxVal, stdVal, startVal, endVal])

    # Entropy-based feature extraction
    @staticmethod
    def EN_extract(self, window): # Completed(?)
        # n_channels = len(window[:,0])
        features = np.array([])
        # print(['CHANNELS:', window])
        # for i in range(n_channels):
        features = np.append(features, stats.entropy(window))
        return features

    # Wavelet transform based feature extraction
    @staticmethod
    def WT_extract(self, window, levels, wname):
        features = np.array([])
        coefficients = pywt.wavedec(window, wname, level = levels)
        for i in range(levels+1):
            # print(['Coeffs: ', coefficients[i]])
            # print(levels)
            minimum = np.amin(coefficients[i])
            maximum = np.amax(coefficients[i])
            stdev = np.std(coefficients[i])
            feats = np.array([minimum, maximum, stdev])
            features = np.append(features, feats)
        features = np.transpose(features)
        return features

    # Auto regressive based feature extraction
    @staticmethod
    def AR_extract(self, window, AROrder):
        r,lg = self.autocorr(window)
                
        features = self.LEVINSON(r[lg>0],order=AROrder,allow_singularity=True)
        return features

    @staticmethod
    def MEAN_extract(self, window):
        #print('hi')
        #print(window)
        return (1.0 * np.sum((window)))/self.numel(window)

    @staticmethod
    def LAST_extract(self, window):
        return window[-1]

    ##############################################################

    ## Class Methods #############################################
    # Constructor:
    def __init__(self, featureOptions={}):
        self.featureOptions = featureOptions
        self.isReady=False

    def __str__(self):
        out='FeatureExtractor with options: '+str(self.featureOptions)
        out=out+'\n'+'Header: '+str(self.header)
        return out

    def extract(self, multiChannelWindow):

        if (self.isReady==False): #The header is not configured yet
            windowColumns=multiChannelWindow.shape[1]
            self.channels=windowColumns
            self.configureHeader()
            self.isReady=True

        N_features = len(self.features_header)
        featureVector = np.array([])

        features_start = 1
        for channel_i in range(self.channels):

            feats = np.array([])
            if self.featureOptions.get('AR'):
                AROrder=self.featureOptions.get('AROrder')
                feats= np.append(feats, FeatureExtractor.AR_extract(self, multiChannelWindow[:,channel_i],AROrder))

            if self.featureOptions.get('EN'):
                feats=np.append(feats, FeatureExtractor.EN_extract(self, multiChannelWindow[:,channel_i]))

            if self.featureOptions.get('TD'):
                feats=np.append(feats, FeatureExtractor.TD_extract(self, multiChannelWindow[:,channel_i]))

            if self.featureOptions.get('STAT'):
                feats=np.append(feats, FeatureExtractor.STAT_extract(self, multiChannelWindow[:,channel_i]))

            if self.featureOptions.get('MEAN'):
                feats=np.append(feats, FeatureExtractor.MEAN_extract(self, multiChannelWindow[:,channel_i]))

            if self.featureOptions.get('WT'):
                levels=self.featureOptions.get('WTLevels')
                wname=self.featureOptions.get('WTName')
                feats=np.append(feats, FeatureExtractor.WT_extract(self,multiChannelWindow[:,channel_i],levels,wname))

            if self.featureOptions.get('LAST'):
                feats=np.append(feats, FeatureExtractor.LAST_extract(self, multiChannelWindow[:,channel_i]))

            if type(featureVector) == np.ndarray and len(featureVector) == 0:
                featureVector = feats;
            else:
                featureVector = np.hstack((featureVector, feats))

            features_start=features_start+self.numel(feats)

        return np.transpose(featureVector)

    ##############################################################

    # HELPER METHODS #############################################

    # Configure Header ###########################################
    def configureHeader(self, channels=None):
        # Set up the header for the feature extractor by looking through the
        # featureoptions and number of channels.
        # The header shows what information is contained at each index of the
        # feature vector.
        # Header is dependent on the type of features and number of channels
        # previously set in the extractor.
        # Optionally you can pass the list of channels so that it does not use
        # generic ones ('CH_1','CH_2'...)
        ########################### WARNING ###############################
        # The order of the header produced by this function is dependent
        # on the parse function order in FeatureExtractor.m.
        # The labels of the header produced by this function are dependent
        # on the features extracted at each specific algorithm (e.g. TC_extract.m)
        ###################################################################
        #
        # Remember that featureOptions could have the following:
        # Names={'TD','STAT', 'AR','AROrder','WT','WTLevels','WTname','EN'};

        channel_headers = np.array([])
        if channels is None:
            for i in range(self.channels):
                channel_headers=np.append(channel_headers, 'CH{}'.format(i))
        else:
            channel_headers=np.array(channels)
            self.channels=len(channel_headers)            

        featureOptions = self.featureOptions

        header=np.array([])
        if (featureOptions.get('AR')):
            AROrder = featureOptions.get('AROrder')
            header = np.concatenate((header,self.AR_header(AROrder)))

        if (featureOptions.get('EN')):
            header = np.concatenate((header,self.EN_header()))

        if (featureOptions.get('TD')):
            header = np.concatenate((header,self.TD_header()))

        if (featureOptions.get('STAT')):
            header = np.concatenate((header,self.STAT_header()))

        if (featureOptions.get('MEAN')):
            header = np.concatenate((header,self.MEAN_header()))

        if (featureOptions.get('WT')):
            WTLevels = featureOptions.get('WTLevels')
            # self.WT_header(WTLevels);
            header = np.concatenate((header,self.WT_header(WTLevels)))

        if (featureOptions.get('LAST')):
            header = np.concatenate((header,self.LAST_header()))

        self.features_header=header

        header=np.array([])
        for i in range(0,len(channel_headers)):
            for j in range(0,len(self.features_header)):
                header=np.append(header,channel_headers[i]+'_'+self.features_header[j])
        self.header=header
        self.isReady=True # Activate the flag to inform that the header is already configured


    # Configure Header: functions for creating each extractor's header

    def AR_header(self, AROrder):
        header = ["" for x in range(AROrder)]
        for i in range(AROrder):
            header[i] = 'a_{}'.format(i)
        return header

    def EN_header(self):
        return ['entropy']

    def TD_header(self):
        return['rms','mean','zeroCross','slopeSignChange','waveformLength','min', 'max', 'std']

    def STAT_header(self):
        return['mean','min', 'max', 'std', 'end']
        # return['mean','min', 'max', 'std', 'start', 'end']

    def LAST_header(self):
        return ['last']

    def MEAN_header(self):
        return ['mean']

    def WT_header(self,levels):
        header = np.array([])
        header = np.append(header, 'min_CA_{}'.format(levels))
        header = np.append(header, 'max_CA_{}'.format(levels))
        header = np.append(header, 'stddev_CA_{}'.format(levels))
        for k in range(levels-1, -1, -1):
            header = np.append(header, 'min_CD_{}'.format(k))
            header = np.append(header, 'max_CD_{}'.format(k))
            header = np.append(header, 'stddev_CD_{}'.format(k))

        return header

    # MATLAB-like functions ######################################
    def numel(self, x):
        if type(x) is np.ndarray:
            if DEBUG:
                print('List.')        
            if type(x[0]) is np.ndarray:
                return len(x) * len(x[0])
            else:
                return len(x)
        else:
            return 1

    def autocorr(self, x):
        r=np.correlate(x,x,mode='full')/len(x)
        lag=np.arange(-len(x)+1,len(x),1)
        return r, lag
        
    
    def LEVINSON(self,r, order=None, allow_singularity=False):    
        #from numpy import isrealobj
        T0  = np.real(r[0])
        T = r[1:]
        M = len(T)
        if order is None:
            M = len(T)
        else:
            assert order <= M, 'order must be less than size of the input data'
            M = order

        realdata = np.isrealobj(r)
        if realdata is True:
            A = np.zeros(M, dtype=float)
            ref = np.zeros(M, dtype=float)
        else:
            A = np.zeros(M, dtype=complex)
            ref = np.zeros(M, dtype=complex)

        P = T0

        for k in range(0, M):
            save = T[k]
            if k == 0:
                temp = -save / P
            else:
                #save += sum([A[j]*T[k-j-1] for j in range(0,k)])
                for j in range(0, k):
                    save = save + A[j] * T[k-j-1]
                temp = -save / P
            if realdata:
                P = P * (1. - temp**2.)
            else:
                P = P * (1. - (temp.real**2+temp.imag**2))
            if P <= 0 and allow_singularity==False:
                raise ValueError("singular matrix")
            A[k] = temp
            ref[k] = temp # save reflection coeff at each step
            if k == 0:
                continue

            khalf = (k+1)//2
            if realdata is True:
                for j in range(0, khalf):
                    kj = k-j-1
                    save = A[j]
                    A[j] = save + temp * A[kj]
                    if j != kj:
                        A[kj] += temp*save
            else:
                for j in range(0, khalf):
                    kj = k-j-1
                    save = A[j]
                    A[j] = save + temp * A[kj].conjugate()
                    if j != kj:
                        A[kj] = A[kj] + temp * save.conjugate()

        return A, P, ref



    ##############################################################

