import rospy
from rospy import ROSException


import warnings

from custom_msgs.msg import String
from std_msgs.msg import Header

from subprocess import Popen,PIPE
import signal
import os


class Recorder(object):
	''' Recorder class for easy rosbag record from python
	Supporting recording on a remote host by sending a command
	to /recorder/command topic using a recorder node that subscribes 
	to commands	
	'''

	def __init__(self,topic='/recorder/command',**kwargs):
		super(Recorder,self).__init__(**kwargs)
		self.topic=topic
		self.pub=None

	def record(self,path=None):
		'''Record rosbag remotely by sending commands to a recorder node'''
		if (self.pub==None):
			self.pub=rospy.Publisher(self.topic, String, queue_size=10)
		if path is None:
			command='record'
			print("Starting record on remote")
		else:
			command='record '+path
			print("Starting record on remote in "+path)
		t=rospy.Time.now()
		strmsg=String()
		strmsg.header=Header(stamp=t)
		strmsg.data=command
		self.pub.publish(strmsg)

	def stop(self):
		print('Stopping record on remote')
		command='stop'
		t=rospy.Time.now()
		strmsg=String()
		strmsg.header=Header(stamp=t)
		strmsg.data=command
		self.pub.publish(strmsg)
