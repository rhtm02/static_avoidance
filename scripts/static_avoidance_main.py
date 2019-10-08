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
	self.SeekDistance = 4.0
	self.INIT = 0
	self.pub = rospy.Publisher('ctrl_cmd', AckermannDriveStamped, queue_size=10)
	self.pub_flag = rospy.Publisher('static_finish', StaticControl, queue_size=10)
	self.flag = StaticControl()
	self.status = 0
    def distance(self, point):
        #point : vector 
	#print("(%f,%f)" %(point.x,point.y))
	dist = math.sqrt((point.x)**2 + (point.y)**2)
	return dist
    def innerProduct(self,point):
        #point : vector
        distA = 1.0
        distB = self.distance(point)
        ip = point.x * 0 + point.y * 1
        ip2 = distA * distB

        CosValue = ip / ip2
        x = math.acos(CosValue)
        degree = math.degrees(x)
    
        return degree

    def SegmentToPolar(self, segment):
	        
        
	center_point = Point()
        
	center_point.x = (segment.first_point.x + segment.last_point.x) / 2
	center_point.y = (segment.first_point.y + segment.last_point.y) / 2
	dist = self.distance(center_point)
	_,angle = self.calSteeringAngle(center_point)
	return[dist,angle]
	
    def LeftSegment(self, segments):
	count = 0
	firstpoint_x = 0
	lastpoint_x = 0
	firstpoint_y = 0
	lastpoint_y = 0
        segment = SegmentObstacle()
        for i in segments:
            Polar = self.SegmentToPolar(i)
            if((Polar[0] <  5) and (Polar[1] > 0)):
                count += 1				
		firstpoint_x += i.first_point.x
		firstpoint_y += i.first_point.y
		lastpoint_x += i.last_point.x
		lastpoint_y += i.last_point.y
	segment.first_point.x = firstpoint_x/count
	segment.first_point.y = firstpoint_y/count
	segment.last_point.x = lastpoint_x/count
	segment.last_point.y = lastpoint_y/count 
        return segment
    def RightSegment(self, segments):
	count = 0
	firstpoint_x = 0
	lastpoint_x = 0
	firstpoint_y = 0
	lastpoint_y = 0
        segment = SegmentObstacle()
        for i in segments:
            Polar = self.SegmentToPolar(i)
            if((Polar[0] <  5) and (Polar[1] < 0)):
                count += 1				
		firstpoint_x += i.first_point.x
		firstpoint_y += i.first_point.y
		lastpoint_x += i.last_point.x
		lastpoint_y += i.last_point.y
	segment.first_point.x = firstpoint_x/count
	segment.first_point.y = firstpoint_y/count
	segment.last_point.x = lastpoint_x/count
	segment.last_point.y = lastpoint_y/count 
        return segment 


    def findNearestSegment(self, segments):
	print("%d Segments detected" % len(segments))
	center_point = Point()
	dist = 0
	for i in segments:
		center_point.x = (i.first_point.x + i.last_point.x) / 2
		center_point.y = (i.first_point.y + i.last_point.y) / 2
		dist = self.distance(center_point)
		if( self.DISTANCE > dist):
               		self.NearestSegmentCenter = Point((i.first_point.x + i.last_point.x) / 2,(i.first_point.y + i.last_point.y) / 2,0 )
			self.NearestSegment = i
			self.DISTANCE = dist
			
		print("NEAREST X,Y : %f, %f" % (self.NearestSegmentCenter.x, self.NearestSegmentCenter.y))
    def calSteeringAngle(self, waypoint):
        angle = math.degrees(math.atan2(waypoint.y, waypoint.x - 0.5))

        steering = -(((20/math.pi) * angle)) + 2
	if (steering <= -28):
		steering = -28
	elif (steering >= 28):
		steering = 28	
        return steering,angle
    def DetectForward(self, segments):
	segment = SegmentObstacle()
	x = 0
	y = 0
	
	for i in segments:
		x = (i.first_point.x + i.last_point.x)/2
		y = (i.first_point.y + i.last_point.y)/2

		_, angle =self.calSteeringAngle(Point(x,y,0))
		print("angle : " + str(angle))
		dist = self.distance(Point(x,y,0))
		if ((angle > -10) and (angle < 10)) and (dist < 10):
			segment.last_point.x = i.last_point.x
			segment.last_point.y = i.last_point.y
			segment.first_point.x = i.first_point.x
			segment.first_point.y = i.first_point.y
	return segment

    def exect(self):
	print("GIVEN START TOPIC")
	self.pub_flag.is_finish = False
	rospy.Subscriber('raw_obstacles', Obstacles, self.obstacle_cb)
    
    def obstacle_cb(self,raw_obstacles):
        print("START CALLBACK")

        self.acker_data.drive.speed = 2.3
        self.acker_data.drive.steering_angle = 0
        self.findNearestSegment(raw_obstacles.segments)
       	
	_, cal_angle = self.calSteeringAngle(self.NearestSegmentCenter)
	print("NEAREST SEGMENT ANGLE : " + str(cal_angle))
	print("DISTANCE : %f" % self.DISTANCE)
	if self.status == 0:	 
            if (cal_angle < 0) and (self.DISTANCE < self.SeekDistance):
                self.status = 7 # left first
	    elif (cal_angle > 0)  and (self.DISTANCE < self.SeekDistance):
		self.status = 1 #right first
# first obstacle is on left 	
	if self.status == 1: 
            steering, _ = self.calSteeringAngle(self.NearestSegmentCenter)
            self.acker_data.drive.steering_angle = steering
            print("STATUS : " + str(self.status))
            print("NearestSegmentCenter : " + str(self.NearestSegmentCenter))
            if (self.DISTANCE <= self.SeekDistance):
                self.status = 2
                self.NearestSegmentCenter = Point(100,100,0)
                self.DISTANCE = 100
                self.INIT = 0
	        self.angle = 0
        elif self.status == 2:

            segment = self.LeftSegment(raw_obstacles.segments)
            tempAngle = 0
            SegmentVector = Point(segment.last_point.x - segment.first_point.x, segment.last_point.y - segment.first_point.y,0)
            if (self.INIT == 0):
                tempAngle = self.innerProduct(SegmentVector)
            else:
                tempAngle = self.angle
            self.angle = self.innerProduct(SegmentVector)
	    print("SEGMENT : " + str(segment))        
            print("STATUS : " + str(self.status))
            print("ANGLE : " + str(self.angle))
            print("TEMPANGLE : " + str(tempAngle))
	    print("angle - tempAngle : " + str(abs(self.angle - tempAngle)))
            self.INIT = 1
	    self.acker_data.drive.steering_angle = 25
	    if (abs(self.angle - tempAngle) >= 15):
                self.status = 3
	        self.NearestSegmentCenter = Point(100,100,0)
                self.DISTANCE = 100
                self.INIT = 0
	        self.angle = 0
        elif self.status == 3:
            wayPoint = Point()
            segment = self.LeftSegment(raw_obstacles.segments)
            wayPoint.x = (segment.first_point.x +segment.last_point.x) / 2
            wayPoint.y = (segment.first_point.y +segment.last_point.y) / 2 - 2.0

            self.acker_data.drive.steering_angle, angle = self.calSteeringAngle(wayPoint)

            print("STATUS : " + str(self.status))
            print("WAYPOINT.X : " + str(wayPoint.x))
            print("WAYPOINT.Y : " + str(wayPoint.y))
            if (wayPoint.x < 0.5):
                self.status = 4
	        self.NearestSegmentCenter = Point(100,100,0)
                self.DISTANCE = 100
                self.INIT = 0
	        self.angle = 0
	elif self.status == 4:
    	    forward_segment = self.DetectForward(raw_obstacles.segments)
	    center = Point((forward_segment.last_point.x + forward_segment.first_point.x)/2,(forward_segment.last_point.y + forward_segment.first_point.y)/2,0)
            steering, _ = self.calSteeringAngle(center)
            self.acker_data.drive.steering_angle = steering
	    dist = self.distance(center)
            print("STATUS : " + str(self.status))
            print("NearestSegmentCenter : " + str(center))
	    print("DISTANCE : " + str(dist))
            if (dist <= self.SeekDistance):
                self.status = 5
                self.NearestSegmentCenter = Point(100,100,0)
                self.DISTANCE = 100
                self.INIT = 0
	        self.angle = 0
        elif self.status == 5:
            segment = self.RightSegment(raw_obstacles.segments)
            tempAngle = 0
            SegmentVector = Point(segment.last_point.x - segment.first_point.x, segment.last_point.y - segment.first_point.y,0)
            if (self.INIT == 0):
                tempAngle = self.innerProduct(SegmentVector)
            else:
                tempAngle = self.angle
            self.angle = self.innerProduct(SegmentVector)
	    print("SEGMENT : " + str(segment))        
            print("STATUS : " + str(self.status))
            print("ANGLE : " + str(self.angle))
            print("TEMPANGLE : " + str(tempAngle))
	    print("angle - tempAngle : " + str(abs(self.angle - tempAngle)))
            self.INIT = 1
	    self.acker_data.drive.steering_angle = 25
	    if (abs(self.angle - tempAngle) >= 15):
                self.status = 6
	        self.NearestSegmentCenter = Point(100,100,0)
                self.DISTANCE = 100
                self.INIT = 0
	        self.angle = 0
        elif self.status == 6:
            wayPoint = Point()
            segment = self.RightSegment(raw_obstacles.segments)
            wayPoint.x = (segment.first_point.x +segment.last_point.x) / 2
            wayPoint.y = (segment.first_point.y +segment.last_point.y) / 2 + 2.0

            self.acker_data.drive.steering_angle, angle = self.calSteeringAngle(wayPoint)

            print("STATUS : " + str(self.status))
            print("WAYPOINT.X : " + str(wayPoint.x))
            print("WAYPOINT.Y : " + str(wayPoint.y))
            if (wayPoint.x < 0.5):
                self.status = 11
	        self.NearestSegmentCenter = Point(100,100,0)
                self.DISTANCE = 100
                self.INIT = 0
	        self.angle = 0
#first obstacle is on right
        elif self.status == 7:
            wayPoint = Point()
            segment = self.RightSegment(raw_obstacles.segments)
            wayPoint.x = (segment.first_point.x +segment.last_point.x) / 2
            wayPoint.y = (segment.first_point.y +segment.last_point.y) / 2 + 2.0

            self.acker_data.drive.steering_angle, angle = self.calSteeringAngle(wayPoint)

            print("STATUS : " + str(self.status))
            print("WAYPOINT.X : " + str(wayPoint.x))
            print("WAYPOINT.Y : " + str(wayPoint.y))
            if (wayPoint.x < 0.5):
                self.status = 8
	        self.NearestSegmentCenter = Point(100,100,0)
                self.DISTANCE = 100
                self.INIT = 0
	        self.angle = 0            
        elif self.status == 8:
    	    forward_segment = self.DetectForward(raw_obstacles.segments)
	    center = Point((forward_segment.last_point.x + forward_segment.first_point.x)/2,(forward_segment.last_point.y + forward_segment.first_point.y)/2,0)
            steering, _ = self.calSteeringAngle(center)
            self.acker_data.drive.steering_angle = steering
	    dist = self.distance(center)
            print("STATUS : " + str(self.status))
            print("FrontSegmentCenter : " + str(center))
	    print("DISTANCE : " + str(dist))
            if (dist <= self.SeekDistance):
                self.status = 9
                self.NearestSegmentCenter = Point(100,100,0)
                self.DISTANCE = 100
                self.INIT = 0
	        self.angle = 0
	elif self.status == 9:
            segment = self.LeftSegment(raw_obstacles.segments)
            tempAngle = 0
            SegmentVector = Point(segment.last_point.x - segment.first_point.x, segment.last_point.y - segment.first_point.y,0)
            if (self.INIT == 0):
                tempAngle = self.innerProduct(SegmentVector)
            else:
                tempAngle = self.angle
            self.angle = self.innerProduct(SegmentVector)
	    print("SEGMENT : " + str(segment))        
            print("STATUS : " + str(self.status))
            print("ANGLE : " + str(self.angle))
            print("TEMPANGLE : " + str(tempAngle))
	    print("angle - tempAngle : " + str(abs(self.angle - tempAngle)))
            self.INIT = 1
	    self.acker_data.drive.steering_angle = 25
	    if (abs(self.angle - tempAngle) >= 15):
                self.status = 10
	        self.NearestSegmentCenter = Point(100,100,0)
                self.DISTANCE = 100
                self.INIT = 0
	        self.angle = 0
	elif self.status == 10:
            wayPoint = Point()
            segment = self.LeftSegment(raw_obstacles.segments)
            wayPoint.x = (segment.first_point.x +segment.last_point.x) / 2
            wayPoint.y = (segment.first_point.y +segment.last_point.y) / 2 - 2.0

            self.acker_data.drive.steering_angle, angle = self.calSteeringAngle(wayPoint)

            print("STATUS : " + str(self.status))
            print("WAYPOINT.X : " + str(wayPoint.x))
            print("WAYPOINT.Y : " + str(wayPoint.y))
            if (wayPoint.x < 0.5):
                self.status = 11
	        self.NearestSegmentCenter = Point(100,100,0)
                self.DISTANCE = 100
                self.INIT = 0
	        self.angle = 0
          
	elif self.status == 11:
		print("END")
		
		self.flag.is_finish.data = True
		self.pub_flag.publish(self.flag)
		rospy.signal_shutdown("END")	
		self.DISTANCE = 100
		self.NearestSegementCenter = Point() 
	if ((self.acker_data.drive.steering_angle > 17) or(self.acker_data.drive.steering_angle < -17)):
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
	sub = rospy.Subscriber('static_operate', StaticControl, play)
        
	rospy.spin()
    except rospy.ROSInterruptException:
        print(error)
        pass
