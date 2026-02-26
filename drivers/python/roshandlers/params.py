import rospy
from rospy import ROSException

import warnings

#from common.dict import getleafs_paths

#API functions for setting and getting parameters
#rospy.set_param(param_name,param_value)
#rospy.get_param(param_name, default=<rospy.client._Unspecified object>)

#For multiple parameters

#{'x':1,'y':2,'sub':{'z':3}}
#will set param_name/x=1, param_name/y=2, and param_name/sub/z=3. Furthermore, it will replace all existing parameters in the param_name namespace with the parameters in param_value. You must set parameters individually if you wish to perform a union update.



# Class for managing parameters from and to the rosparam server
# an instance reads a param name (it could be not a leaf parameter i.e. dictionary of parameters)
# for example, you can create the object:
# param1=ROSParams('/EarlyStance/Knee/')

# Then, the param1 object has a dictionary for all the parameters under '/EarlyStance/Knee'
# If there are no parameters in the param server the dictionary is empty.
# You can modify the dictionary to change the parameter values and when done you can upload the parameters to the parameter server



class Rosparam(object):

    def __init__(self,name):
        self.name=name
        self.download()

    def set(self, param_name,param_value):
    # set a parameter directly in the parameter server
        return rospy.set_param(param_name,param_value)

    def get(self, name):
    # get a parameter directly from the parameter server
        try:
            return rospy.get_param(name)
        except ROSException:
            warnings.warn("ROSException")
        except KeyError:
            warnings.warn("%s: parameter does not exist in rosmaster"%name)
            return []

    def upload(self):
    # upload the parameters stored in this instance to rosmaster
        return rospy.set_param(self.name,self.dictionary)

    def download(self):
    #download the parameters from rosmaster to the dictionary in this instance
        try:
            self.dictionary=rospy.get_param(self.name)
        except ROSException:
            warnings.warn("ROSException")
        except KeyError:
            warnings.warn("%s: parameter does not exist in rosmaster"%self.name)
            self.dictionary=[]
