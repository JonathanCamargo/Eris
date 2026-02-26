import warnings
from rospy import ROSInitException
import rospy

class ROSConnector:
    ros_ok=False
    def __init__(self,**kwargs):
       pass

    def connect(self):
        self.ros_ok=False
        try:
            print("Connecting to rosmaster")
            rospy.init_node('listener', anonymous=True,disable_signals=True)
            print("Found these topics:"+str(rospy.get_published_topics()))
            print("Connected to rosmaster")
            self.ros_ok=True
        except ROSInitException:
	        warnings.warn("Could not initialize ros node")
	        self.ros_ok=False
        finally:
            pass
        return self.ros_ok

    def isOK(self):
        return self.ros_ok

    def get_published_topics(self):
	    return rospy.get_published_topics()

    def shutdown(self,reason='SIGNAL'):
	    rospy.signal_shutdown('SIGNAL')
