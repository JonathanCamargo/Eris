import rospy
from rospy import ROSException


import warnings

from subprocess import Popen,PIPE
import signal
import os


class Rosbag(object):
	''' A rosbag class for recording data from python'''
	pid=None #pid for rosbag process
	path="/home/default.bag"
	process=None

	def __init__(self,path="/home/default.bag",topics=None,**kwargs):
		super(Rosbag,self).__init__(**kwargs)
		self.path=path
		self.topics=topics

	def record(self,path=None,duration=None):
		'''Record a bag file in a path duration is a numeric value
		in minutes.
		'''
		if path is None: #Using default path
			path=self.path
		if self.topics==None: #Record all topics if a list of topics is not given
			topicslist=['-a']
		else:
			topicslist=self.topics

		arg=["rosbag", "record"]
		if duration is not None:
			arg.append("--duration="+str(duration)+"m")
		for topic in topicslist:
			arg.append(topic)
		arg.append("-O")
		arg.append(self.path)
		arg.append('-b')
		arg.append('0')
		print(arg)

		#Record rosbag in another process
		self.process=Popen(arg)
		self.pid=self.process.pid
		print("Recording rosbag to %s (pid %s)"%(self.path,self.pid))

	def stop(self):
		#kill self.pid
		print("Terminating rosbag with pid %s"%self.pid)
		ps_command = Popen("ps -o pid --ppid %d --noheaders" % self.pid,shell=True, stdout=PIPE)
		ps_output = ps_command.stdout.read()
		ps_output = ps_output.decode('utf-8')
		retcode = ps_command.wait()
		if not(retcode==0):
			print("problem with ps")
		else:
			for pid_str in ps_output.split("\n")[:-1]:
				os.kill(int(pid_str), signal.SIGINT)
				self.process.send_signal(signal.SIGINT)
