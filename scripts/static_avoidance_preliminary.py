#!/usr/bin/env python

import rospy
import math

from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import SegmentObstacle 
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from ackermann_msgs.msg import AckermannDriveStamped
from static_avoidance.msg import StaticControl
from std_msgs.msg import Bool

class StaticAvoidance:

    def __init__(self):
        #INIT 
	self.NearestSegment = SegmentObstacle()
	self.acker_data = AckermannDriveStamped()
	self.NearestSegmentCenter = Point()
	self.DISTANCE = 100.0
	self.angle = 0
	self.cal_angle = 0
	self.SeekDistance = 3.0
	
	self.pub = rospy.Publisher('ctrl_cmd', AckermannDriveStamped, queue_size=10)
	self.pub_flag = rospy.Publisher('end', StaticControl, queue_size=10)
	self.flag = StaticControl()
	self.status = 0
    def distance(self, point):
        #point : vector 
	#print("(%f,%f)" %(point.x,point.y))
	dist = math.sqrt((point.x)**2 + (point.y)**2)
	return dist

    def SegmentToPolar(self, segments):
        
        segmentPolar_list = []
	center_point = Point()
        for i in segments:
		center_point.x = (i.first_point.x + i.last_point.x) / 2
		center_point.y = (i.first_point.y + i.last_point.y) / 2
		dist = self.distance(center_point)
		_,angle = self.calSteeringAngle(center_point)
		segmentPolar_list.append([dist,angle])
	return segmentPolar_list
		
    
    def findNearestSegment(self, segments):
	print("%d Segments detected" % len(segments))
	center_point = Point()
	dist = 0
	for i in segments:
		center_point.x = (i.first_point.x + i.last_point.x) / 2
		center_point.y = (i.first_point.y + i.last_point.y) / 2
		dist = self.distance(center_point)
		print("dist %d" % dist)
		print("CENTER POINT (%f , %f)" % (center_point.x,center_point.y))
		if( self.DISTANCE > dist):
			print("CHANGE")
               		self.NearestSegmentCenter = Point((i.first_point.x + i.last_point.x) / 2,(i.first_point.y + i.last_point.y) / 2,0 )
			self.NearestSegment = i
			self.DISTANCE = dist
			
		print("NEAREST X,Y : %f, %f" % (self.NearestSegmentCenter.x, self.NearestSegmentCenter.y))
    def calSteeringAngle(self, waypoint):
        angle = math.degrees(math.atan2(waypoint.y, waypoint.x - 0.5))

        steering = -(((28/math.pi) * angle)) + 2
	if (steering <= -28):
		steering = -28
	elif (steering >= 28):
		steering = 28
	
        return steering,angle
    def exect(self):
	print("GIVEN START TOPIC")
	rospy.Subscriber('raw_obstacles', Obstacles, self.obstacle_cb)
    
    def obstacle_cb(self,raw_obstacles):
        print("START CALLBACK")

        self.acker_data.drive.speed = 2.3
        self.acker_data.drive.steering_angle = 0
        self.findNearestSegment(raw_obstacles.segments)
       	
	print("NEAREST X,Y : %f, %f" % (self.NearestSegmentCenter.x, self.NearestSegmentCenter.y))
	_, self.cal_angle=self.calSteeringAngle(self.NearestSegmentCenter)
	print("NEAREST SEGMENT ANGLE : " + str(self.cal_angle))
	print("NEAREST X,Y : %f, %f" % (self.NearestSegmentCenter.x, self.NearestSegmentCenter.y))
	print("DISTANCE : %f" % self.DISTANCE)
	if self.status == 0:	 
		if (self.cal_angle < 0) and (self.DISTANCE < self.SeekDistance):
			self.status = 4
		elif(self.cal_angle >= 0) and (self.DISTANCE < self.SeekDistance):
			self.status = 1
			 
	elif self.status == 1:	#obstacle is located on left
		wayPoint = Point()
		wayPoint.x = self.NearestSegmentCenter.x		
		wayPoint.y = self.NearestSegmentCenter.y - 1.5
		steering,_ = self.calSteeringAngle(wayPoint)		
		self.acker_data.drive.steering_angle = steering
		print("X,Y : %f, %f" % (wayPoint.x, wayPoint.y))
		if (self.cal_angle > 80):
			self.status = 2		
			self.DISTANCE = 100
			self.NearestSegementCenter = Point() 
			
	elif self.status == 2:
		self.acker_data.drive.steering_angle = -28		
		if ((self.cal_angle < 0) and (self.DISTANCE < 3)):
			self.status = 3 
			
	elif self.status == 3:
		wayPoint = Point()
		wayPoint.x = self.NearestSegmentCenter.x		
		wayPoint.y = self.NearestSegmentCenter.y + 1.5
		steering, _ = self.calSteeringAngle(wayPoint)	
		print("X,Y : %f, %f" % (wayPoint.x, wayPoint.y))	
		self.acker_data.drive.steering_angle = steering
		if (self.cal_angle < -75) and (self.DISTANCE < 1.0):		
			self.DISTANCE = 100
			self.NearestSegementCenter = Point() 
			self.status = 6		
			
	elif self.status == 4:	#obstacle is located on right
		wayPoint = Point()
		wayPoint.x = self.NearestSegmentCenter.x		
		wayPoint.y = self.NearestSegmentCenter.y + 1.5
		steering,_ = self.calSteeringAngle(wayPoint)		
		self.acker_data.drive.steering_angle = steering
		print("X,Y : %f, %f" % (wayPoint.x, wayPoint.y))
		if (self.cal_angle < -80):
			self.status = 5			
			self.DISTANCE = 100
			self.NearestSegementCenter = Point() 
			
		
	elif self.status == 5:
		data_lst = self.SegmentToPolar(raw_obstacles.segments)
		data = []
		wayPoint = Point()
		print(data_lst)
		for i in data_lst:
			data = i
			if ((i[0] < 3) and (i[1] > -10)):
				data = i
		print(data)
		wayPoint.x = data[0] * math.cos(math.radians(data[1]))
		wayPoint.y = data[0] * math.sin(math.radians(data[1])) - 1.5
		steering,_ = self.calSteeringAngle(wayPoint)
		self.acker_data.drive.steering_angle = steering
		print("X,Y : %f, %f" % (wayPoint.x, wayPoint.y))
		if ((data[1] > 90) and (data[0] < 1.0)):
			self.status = 6		
			self.DISTANCE = 100
			self.NearestSegementCenter = Point() 
			
	elif self.status == 6:
		print("END")
		
		self.flag.is_finish.data = True
		self.pub_flag.publish(self.flag)
		rospy.signal_shutdown("END")	
		self.DISTANCE = 100
		self.NearestSegementCenter = Point() 
	if ((self.acker_data.drive.steering_angle > 17) or  (self.acker_data.drive.steering_angle < -17)):
		self.acker_data.drive.speed = 1.0

	print("STEER : " + str(self.acker_data.drive.steering_angle))
	print("STATUS : " + str(self.status))
	
	self.DISTANCE = 100
	
        self.pub.publish(self.acker_data)
	self.pub_flag.publish(self.flag)

def play(data):
	if (data.is_operate.data == True):
		mission.exect()
	else:
		print("is_operate is not true")


if __name__ == '__main__':
    try:
        rospy.init_node('StaticAvoidance', anonymous=True)
        mission = StaticAvoidance()
	sub = rospy.Subscriber('start', StaticControl, play)
        
	rospy.spin()
    except rospy.ROSInterruptException:
        print(error)
        pass
