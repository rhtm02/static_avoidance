#!/usr/bin/env python

import rospy


#10hz

from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Bool




class test:

	def __init__(self):

		self.pub_talk = rospy.Publisher('talk_2', Int32, queue_size=10)
		self.pub_flag = rospy.Publisher('flag_2', Bool, queue_size=10)
		self.flag = Bool()
		self.talk = Int32() 
		self.count = 0

	def exect(self):
		print("EXECT CALLBACK")
		self.sub = rospy.Subscriber('talk', String, self.cb)


	def cb(self, data):
		if (self.count <= 10):
			print(data)
			self.count += 1
			self.talk.data = self.count
			self.pub_talk.publish(self.talk)
		else:
			self.flag.data = False
			print(self.flag.data)
			self.pub_flag.publish(self.flag)
			rospy.signal_shutdown("END")



def play(data):
	print("IN PLAY")
	print(data.data)
	
	if (data.data == True):
		Mission.exect()
		
	else:
		print("flag is not true")
	


if __name__ == '__main__':
    try:
	rospy.init_node('StaticAvoidance', anonymous=True)
	Mission = test()	
	sub =  rospy.Subscriber('flag', Bool, play)
  	rospy.spin()
	
    except rospy.ROSInterruptException:
        print(error)
        pass
