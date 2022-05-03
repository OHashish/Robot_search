#!/usr/bin/env python
from __future__ import division
import cv2
import rospy
import numpy as np

import yaml

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError

import time
import math


###JUST FOR DEBUGGING
import os


###ACTUATOR CLASS FROM LAB 4
class GoToPose():
	def __init__(self):

		self.goal_sent = False

		# What to do if shut down (e.g. Ctrl-C or failure)
		rospy.on_shutdown(self.shutdown)

		# Tell the action client that we want to spin a thread by default
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("Wait for the action server to come up")

		self.move_base.wait_for_server()

	def goto(self, pos, quat):

		# Send a goal
		self.goal_sent = True
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
									Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
		self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
		success = self.move_base.wait_for_result(rospy.Duration(60)) 

		state = self.move_base.get_state()
		result = False

		if success and state == GoalStatus.SUCCEEDED:
			# We made it!
			result = True
		else:
			self.move_base.cancel_goal()

		self.goal_sent = False
		return result

	def shutdown(self):
		if self.goal_sent:
			self.move_base.cancel_goal()
		rospy.loginfo("Dying")
		rospy.sleep(1)


##class to detect faces
class faceDetector():

	def __init__(self):
		self.face_found = False

	def start_search(self):
		self.t_start = time.time()
		# Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
		self.bridge = CvBridge()

		# We covered which topic to subscribe to should you wish to receive image data
		self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)

	def stop_search(self):
		self.image_sub.unregister()
		#self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback2)
		

	def callback2(self,data):
		#just eat the messages we don't need
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)

	def callback(self,data):
		##get the image
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)


		##convert to grayscale
		
		self.gray_image = cv2.cvtColor(self.cv_image, cv2.COLOR_RGB2GRAY)
		the_path = os.path.expanduser('~/catkin_ws/src/group25/haarcascades/haarcascade_frontalface_default.xml')
		self.cascade_classifier = cv2.CascadeClassifier(the_path)

		detected_objects = self.cascade_classifier.detectMultiScale(self.gray_image)

		#Draw rectangles on the detected objects
		if len(detected_objects) != 0:
			rospy.loginfo('FOUND A FACE')
			self.face_found = True
			# the_image_path = os.path.expanduser('~/catkin_ws/src/group25/output/Cluedo_character.png')
			# cv2.imwrite(the_image_path, self.cv_image)
			for (x, y, width, height) in detected_objects:
				cv2.rectangle(self.cv_image, (x, y),
							(x + height, y + width),
							(0, 255, 0), 2)

		cv2.namedWindow('face')
		cv2.imshow('face', self.cv_image)
		cv2.waitKey(3)

		if time.time() - self.t_start >=4:
			self.stop_search()
		

class colourIdentifier():

	def __init__(self):

		# Initialise any flags that signal a colour has been detected (default to false)
		self.green_found = False
		self.red_found = False
		self.blue_found = False
		self.yellow_found = False
		self.purple_found= False
		self.timeof_last = None

	def start_search(self):
		self.green_found = False
		# Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
		self.bridge = CvBridge()

		# We covered which topic to subscribe to should you wish to receive image data
		self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)


	###only works for red, todo: implement all colours
	def start_face_search(self):
		self.red_found = False
		self.blue_found = False
		self.yellow_found = False
		self.purple_found= False
		self.angle=0
		self.bridge = CvBridge()
		self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
		self.desired_velocity = Twist()
		self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback3)
		self.angle_sub = rospy.Subscriber('/scan', LaserScan, self.get_angle)
		
	def stop_face_search(self):
		self.image_sub.unregister()
	def stop_search(self):
		self.image_sub.unregister()
		self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback2)
		self.green_found = False

	def callback2(self,data):
		#just eat the messages we don't need
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)

	


	def callback(self, data):
		# Convert the received image into a opencv image
		# But remember that you should always wrap a call to this conversion method in an exception handler
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)

		# # Set the upper and lower bounds for the colour you wish to identify - green
		self.hsv_green_lower = np.array([65,52,72])
		self.hsv_green_upper = np.array([75,255,255])

		

		# Convert the rgb image into a hsv image
		self.hsv_img = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

		# Filter out everything but a particular colour using the cv2.inRange() method
		self.mask = cv2.inRange(self.hsv_img,self.hsv_green_lower,self.hsv_green_upper)
		# Apply the mask to the original image using the cv2.bitwise_and() method
		self.result = cv2.bitwise_and(self.cv_image,self.cv_image,mask =self.mask)


		# Find the contours that appear within the certain colour mask using the cv2.findContours() method
		# For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE

		self.contours = cv2.findContours(self.mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[0]


		if len(self.contours) > 0:

			# Loop over the contours
			# There are a few different methods for identifying which contour is the biggest:
			# Loop through the list and keep track of which contour is biggest or
			# Use the max() method to find the largest contour
			c = max(self.contours, key=cv2.contourArea)

			#Moments can calculate the center of the contour
			M = cv2.moments(c)
			try:
				cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
			except ZeroDivisionError:
				print("divzero")

			#Check if the area of the shape you want is big enough to be considered
			# If it is then change the flag for that colour to be True(1)
			if cv2.contourArea(c) > 3500:  #<What do you think is a suitable area?>:
				
				rospy.loginfo(cv2.contourArea(c))
				# draw a circle on the contour you're identifying
				#minEnclosingCircle can find the centre and radius of the largest contour(result from max())
				(x, y), radius = cv2.minEnclosingCircle(c)

				cv2.circle(self.result,(int(x),int(y)),int(radius),[155,50,50],5)

				# Then alter the values of any flags
				self.green_found = True


		#if the flag is true (colour has been detected)
			#print the flag or colour to test that it has been detected
			#alternatively you could publish to the lab1 talker/listener
			if (self.green_found == True):
				rospy.loginfo("Green found!")


		#Show the resultant images you have created. You can show all of them or just the end result if you wish to.
		cv2.namedWindow("dbg_window")
		cv2.imshow("dbg_window",self.cv_image)
		cv2.waitKey(3)

		cv2.namedWindow("window")
		cv2.imshow("window",self.result)

	def callback3(self, data):
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)

		#Scarlett Upper and Lower Bounds
		self.hsv_scar_lower = np.array([0,100,20])
		self.hsv_scar_upper = np.array([5,255,255])

		#Plum Upper and Lower Bounds
		self.hsv_plum_lower = np.array([145,100,20])
		self.hsv_plum_upper = np.array([160,255,255])

		#Mustard Upper and Lower Bounds
		self.hsv_mus_lower = np.array([23,100,20])
		self.hsv_mus_upper = np.array([35,255,255])

		#Peacock Upper and Lower Bounds
		self.hsv_pea_lower = np.array([80,100,20])
		self.hsv_pea_upper = np.array([120,255,255])

		self.hsv_img = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

		#Combine all masks
		self.mask_scar = cv2.inRange(self.hsv_img,self.hsv_scar_lower,self.hsv_scar_upper)
		self.mask_plum = cv2.inRange(self.hsv_img,self.hsv_plum_lower,self.hsv_plum_upper)
		self.mask_mus = cv2.inRange(self.hsv_img,self.hsv_mus_lower,self.hsv_mus_upper)
		self.mask_pea = cv2.inRange(self.hsv_img,self.hsv_pea_lower,self.hsv_pea_upper)
		self.comb_1=cv2.bitwise_or(self.mask_scar,self.mask_plum)
		self.comb_2=cv2.bitwise_or(self.mask_mus,self.mask_pea)
		self.comb=cv2.bitwise_or(self.comb_1,self.comb_2)
		self.result = cv2.bitwise_and(self.cv_image,self.cv_image,mask =self.comb)

		self.contours = cv2.findContours(self.comb,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[0]


		if len(self.contours) > 0:
			cx=0
			cy=0
			c = max(self.contours, key=cv2.contourArea)
			rospy.loginfo(cv2.contourArea(c))
			M = cv2.moments(c)
			try:
				cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
			except ZeroDivisionError:
				print("divzero")
			#Rotate Robot until red object is at the center
			if cv2.contourArea(c) < 15000 and cv2.contourArea(c)>50:
				self.desired_velocity.linear.x = 0
				if cx>330:
					rospy.loginfo("Rotating Right...")
					rospy.loginfo(cx)
					self.desired_velocity.angular.z = -0.075
					for i in range (1):
						self.timeof_last = time.time()
						self.pub.publish(self.desired_velocity)
				if cx<300:
					rospy.loginfo("Rotating Left ...")
					self.desired_velocity.angular.z = 0.075
					for i in range (1):
						self.timeof_last = time.time()
						self.pub.publish(self.desired_velocity)
				self.desired_velocity.angular.z = 0
					
			#If cluedo related color is centered then start moving towards it
			if cv2.contourArea(c) < 14000 and cv2.contourArea(c)>50 and (cx<330 and cx>300):	
				(x, y), radius = cv2.minEnclosingCircle(c)

				cv2.circle(self.result,(int(x),int(y)),int(radius),[155,50,50],5)
				self.desired_velocity.linear.x = 0.3
				for i in range (15):
					self.timeof_last = time.time()
					self.pub.publish(self.desired_velocity)

			elif cv2.contourArea(c) > 16000:
				self.desired_velocity.linear.x = -0.3
				for i in range (15):
					self.timeof_last = time.time()
					self.pub.publish(self.desired_velocity)


			elif cv2.contourArea(c) > 14500:
				
				rospy.loginfo("Sus color found (cLeUDo???)!")
				
				self.desired_velocity.linear.x = 0
				for i in range (10):
					self.pub.publish(self.desired_velocity)
				
				#This is the angle to the wall
				#TODO:Add angle correction so the robot stops acting sus

				print(self.angle)
				# if self.angle>120:
				# 	self.desired_velocity.linear.x = 0
				# 	print(180-self.angle)
				# 	correction=math.radians(180-self.angle)-(math.radians(180-self.angle)*0.4)
				# 	self.desired_velocity.angular.z = correction
				# 	for i in range (10):
				# 		self.pub.publish(self.desired_velocity)
				# 	self.desired_velocity.angular.z=0
					

				##scarlet?
				color_contour = cv2.findContours(self.mask_scar,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[0]
				if len(color_contour) > 0:

					aux = max(color_contour, key=cv2.contourArea)
					if cv2.contourArea(aux) > 14000:
						#found scarlet
						self.red_found = True
				##plum?
				color_contour = cv2.findContours(self.mask_plum,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[0]
				if len(color_contour) > 0:
					aux = max(color_contour, key=cv2.contourArea)
					if cv2.contourArea(aux) > 14000:
						#found plum
						self.purple_found = True
				##mustard?
				color_contour = cv2.findContours(self.mask_mus,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[0]
				
				if len(color_contour) > 0:
					aux = max(color_contour, key=cv2.contourArea)
					if cv2.contourArea(aux) > 14000:
						#found mustard
						self.yellow_found = True
				##peacock?
				color_contour = cv2.findContours(self.mask_pea,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[0]
				if len(color_contour) > 0:
					
					aux = max(color_contour, key=cv2.contourArea)
					if cv2.contourArea(aux) > 14000:
						#found peacock
						self.blue_found = True

				
				
		#Show the resultant images you have created. You can show all of them or just the end result if you wish to.
		cv2.namedWindow("dbg_window")
		cv2.imshow("dbg_window",self.cv_image)
		cv2.waitKey(3)

		cv2.namedWindow("window")
		cv2.imshow("window",self.result)


		# cv2.namedWindow("dbgg")
		# cv2.imshow("dbgg",self.resss)

	def get_angle(self,msg):
		#Calculates angle of wall (any object) relating the robot (middle line)
		#To be used to fix the robot acting sus when approaching wall in a weird(steep) angle 
		#Two angles are calculated because if another object is in the frame it calculates
		#the angle relating to it not the wall
		
		newlist = [x for x in msg.ranges if np.isnan(x) == False]
		val_1=int(math.floor( len(newlist)/4 ))
		val_2=int(math.floor( (len(newlist)/2 )+ (len(newlist)/4) ))
		val_3=int(math.floor( (len(newlist)/2 ) ))

		if newlist[val_1]>newlist[val_2]:
			dist1=newlist[val_1]
		else:
			dist1=newlist[val_2]
		
		dist2=newlist[val_3]
		dist_diff=math.sqrt((((dist1)**2)+((dist2)**2))-(2*(dist1)*(dist2)*np.cos(math.radians(30))))
		angle_1=math.asin((np.sin(math.radians(30))/dist_diff)*dist1)
		ang_to_wall=180-math.degrees(angle_1)

		dist1=max(newlist)
		dist_diff=math.sqrt((((dist1)**2)+((dist2)**2))-(2*(dist1)*(dist2)*np.cos(math.radians(30))))
		angle_1=math.asin((np.sin(math.radians(30))/dist_diff)*dist1)
		ang_to_wall_2=180-math.degrees(angle_1)

		if ang_to_wall>ang_to_wall_2:
			self.angle=ang_to_wall
		else:
			self.angle=ang_to_wall_2
		

###CLASS FOR THE ROBOT
class Bobot():

	def __init__(self, ents = None, mids = None):


		###variable to see if robot is doing anything
		# if the robot does not move for 5 seconds set it to True
		# manage it in function calls
		self.idle = True

		#time of last action
		self.timeof_last = None

		##points
		self.entrance_points = ents
		self.mid_points = mids
		##navigator
		self.navigator = GoToPose()
		##camera 
		self.camera = colourIdentifier()
		##face detector
		self.facer= faceDetector()
		###figured out correct room flag
		self.found_room = False
		self.the_room = None
		self.endGoal = False



	##go sequantially to entrance points
	def go_to_entrances(self):

		for i in range(len(self.entrance_points)):

			##IF ROOM IS FOUND THEN BREAK
			if self.found_room:
				break

			# Customize the following values so they are appropriate for your location
			x = self.entrance_points[i][0]# SPECIFY X COORDINATE HERE
			y = self.entrance_points[i][1]# SPECIFY Y COORDINATE HERE

			theta = 0# SPECIFY THETA (ROTATION) HERE
			position = {'x': x, 'y' : y}
			quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

			rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
			success = self.navigator.goto(position, quaternion)

			if success:
				rospy.loginfo("Hooray, reached the desired pose")

				##take some time off to avoid overspin/skidding
				rospy.sleep(2)

				###LOOK FOR CIRCLE
				self.camera.start_search()

				##do a full spin abusing navigator class

				for n_theta in [np.pi/3, 2/3*np.pi, np.pi, 4/3 * np.pi, 5/3*np.pi, 0]:

					quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(n_theta/2.0), 'r4' : np.cos(n_theta/2.0)}
					rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
					self.navigator.goto(position, quaternion)
					#chill
					rospy.sleep(3)

					##check if green was found
					if self.camera.green_found:

						##stop searching
						self.camera.stop_search()
						self.found_room = True
						self.the_room = self.mid_points[i]
						break

				##stop searching
				self.camera.stop_search()

			else:
				rospy.loginfo("The base failed to reach the desired pose")

			# Sleep to give the last log messages time to be sent
			rospy.sleep(1)

	def go_to_room(self):

		if self.found_room:
			x = self.the_room[0]# SPECIFY X COORDINATE HERE
			y = self.the_room[1]# SPECIFY Y COORDINATE HERE

			theta = 0# SPECIFY THETA (ROTATION) HERE
			position = {'x': x, 'y' : y}
			quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

			rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
			success = self.navigator.goto(position, quaternion)

			if success:
				rospy.loginfo("Hooray, reached the desired pose(We Middle)")
			else:
				rospy.loginfo("The base failed to reach the desired pose")

	def face_search(self):
		##make the robot busy
		self.idle = False
		self.endGoal = False

		self.timeof_last = time.time()
		self.camera.timeof_last = time.time()

		###check inside this function if robot is busy or not
		self.camera.start_face_search()

		while not self.idle:
			#check if robot did nothing for 4 seconds
			#print(time.time() - self.camera.timeof_last)
			if time.time() - self.camera.timeof_last >= 4:
				self.idle = True


		##take a break from all the colour searching
		self.camera.stop_face_search()

		

		##check if face
		if self.camera.blue_found or self.camera.green_found or self.camera.red_found or self.camera.yellow_found:
			
			self.facer.start_search()

			#self.facer.stop_search()	
			rospy.loginfo("ASDFASDFASFD")
			print("HELLOOdsasdfasdfOO???")

			time.sleep(5)
			print(self.facer.face_found)
			if self.facer.face_found:
				##since all is aligned and everything take a screenshot
				rospy.loginfo("taking screenshot")
	
				the_image_path = os.path.expanduser('~/catkin_ws/src/group25/output/cluedo_character.png')
				cv2.imwrite(the_image_path, self.facer.cv_image)
				the_text_path = os.path.expanduser('~/catkin_ws/src/group25/output/cluedo_character.txt')
				f = open(the_text_path, 'w')
				if self.camera.red_found:
					f.write("Scarlet")
					self.endGoal = True
				if self.camera.blue_found:
					f.write("Peacock")
					self.endGoal = True
				if self.camera.yellow_found:
					f.write("Mustard")
					self.endGoal = True
				if self.camera.purple_found:
					f.write("Plum")
					self.endGoal = True
				time.sleep(3)
				return True
				
			
			self.facer.stop_search() 

		cv2.destroyAllWindows()
		return False


	def green_room_traversal(self):

		self.idle = False
		self.traverse_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size = 10)
		
		print('you spin me right round baby right round')

		##similar to lab2 exercise 1		
		traverse_rate = rospy.Rate(10) #10hz

		traversal_velocity = Twist()
		traversal_velocity.linear.x = 0.5
		traversal_velocity.angular.z = 0.5

		last = time.time()

		while self.endGoal == False:

			#self.check_distance_to_obstacle()

			now = time.time()

			## if one second has passed since last stop
			if int(now - last) == 4:
				
				#stop BOTtas
				traversal_velocity.linear.x = 0.0
				traversal_velocity.angular.z = 1
				self.traverse_pub.publish(traversal_velocity)
				
				for i in range(60):
					self.traverse_pub.publish(traversal_velocity)

					#PERFORM CAMERA CHECK FOR IMAGES
					self.face_search()

					if self.endGoal == True:
						rospy.loginfo('BOTtas found the Cluedo Picture')
						traversal_velocity.linear.x = 0.0
						traversal_velocity.angular.z = 0.0
						break
						
					traverse_rate.sleep()
				
				traversal_velocity.linear.x = 0.5
				traversal_velocity.angular.z = 0.5
				last = time.time()

			else:

				self.traverse_pub.publish(traversal_velocity)
				traverse_rate.sleep()


		if rospy.is_shutdown():
			traversal_velocity.linear.x = 0
			traversal_velocity.angular.z = 0
			self.traverse_pub.publish(traversal_velocity)


	def check_distance_to_obstacle(self):

		self.obstacle_sub = rospy.Subscriber('/scan', LaserScan, obstacle_distance)

		rospy.spin()

	def random_points_method(self):
		#TODO:1.Generate random points relating to the robot
		#2.remove any points that are close to each other
		#3.catch exception if the path is out of bounds
		#4.Get path distance and add a reasonable threshold so it doesn't go 
		# to another room (path to room will be very long)
		x = self.mid_points[0][0]# SPECIFY X COORDINATE HERE
		y = self.mid_points[0][1]# SPECIFY Y COORDINATE HERE
		print(x)
		print(y)
		theta = 0# SPECIFY THETA (ROTATION) HERE
		position = {'x': x, 'y' : y}
		quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

		rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
		success = self.navigator.goto(position, quaternion)
		rospy.spin()
		


	
	
	

	
def obstacle_distance(msg):

	print(msg.ranges[360])

	spin_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size = 10)
	spin_velocity = Twist()

	#setting speed to turn around
	spin_velocity.linear.x = 0.0
	spin_velocity.angular.z = 1

	#if the robot is too close to obstacle
	if msg.ranges[360] < 1:
		for p in range(30):	
			spin_pub.publish(spin_velocity)



if __name__ == '__main__':
	try:
		##node
		rospy.init_node('bobot_boy', anonymous=True)

		##NO ARGS = CLEAN RUN; ARGS = DEBUGGING
			##read points from yaml files and do some sorting so coding is easier
		if len(sys.argv) == 1:
			points = []
			the_path = os.path.expanduser('~/catkin_ws/src/group25/world/input_points.yaml')
			with open(the_path, 'r') as stream:
				points = yaml.safe_load(stream)

			##entrances
			ents = []
			ents.append(points['room1_entrance_xy'])
			ents.append(points['room2_entrance_xy'])

			mids = []
			mids.append(points['room1_centre_xy'])
			mids.append(points['room2_centre_xy'])

			##make a depressed little robot (his name is Bobot)
			robot = Bobot(ents,mids)

			robot.go_to_entrances()
			robot.go_to_room()
			robot.green_room_traversal()


		###DEBUGGING SECTION, DELETE BEFORE SUBMISSION
		elif sys.argv[1] == 'debug_face':


			print('DEBUGGING FACE STUFF')
			facer = faceDetector()
			facer.start_search()
			print("MAMAMAMAMAMA")
			rospy.spin()
		elif sys.argv[1] == 'debug_face_search':


			print('DEBUGGING FACE Search STUFF')
			facerc = colourIdentifier()
			facerc.start_face_search()
			
			# robot = Bobot()
			# robot.face_search()
			# print("spam")
			rospy.spin()

		elif sys.argv[1] == 'circle_room':

			BOTtas = Bobot()
			print("BOTtas is born")
			print('DEBUGGING GREEN ROOM TRAVERSAL')
			try:
				BOTtas.green_room_traversal()
			except rospy.ROSInterruptException:
				pass
		
		elif sys.argv[1] == 'random_room':

			points = []
			the_path = os.path.expanduser('~/catkin_ws/src/group25/world/input_points.yaml')
			with open(the_path, 'r') as stream:
				points = yaml.safe_load(stream)

			mids = []
			mids.append(points['room1_centre_xy'])
			mids.append(points['room2_centre_xy'])

			##make a depressed little robot (his name is Bobot)
			robot = Bobot(ents,mids)

			print("BOTtas is born")
			print('DEBUGGING RANDOM ROOM ')
			try:
				robot.random_points_method()
			except rospy.ROSInterruptException:
				pass

		elif sys.argv[1] == 'cluedo_identifier':

			BOBBot = Bobot()
			BOBBot.face_search()
			print("Done?")
			

			


	except rospy.ROSInterruptException:
		rospy.loginfo("Ctrl-C caught. Quitting")

	#cv2.destroyWindow('window')


