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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

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
		rospy.loginfo("Stop")
		rospy.sleep(1)


##class to detect faces
class faceDetector():

	def __init__(self):
		self.face_found = False

	def start_search(self):
		self.green_found = False
		# Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
		self.bridge = CvBridge()

		# We covered which topic to subscribe to should you wish to receive image data
		self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)

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

	def callback(self,data):
		##get the image
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)


		##convert to grayscale
		
		self.gray_image = cv2.cvtColor(self.cv_image, cv2.COLOR_RGB2GRAY)

		self.cascade_classifier = cv2.CascadeClassifier('./../haarcascades/haarcascade_frontalface_default.xml')

		detected_objects = self.cascade_classifier.detectMultiScale(self.gray_image,1.3,5)

		#Draw rectangles on the detected objects
		if len(detected_objects) != 0:
			print('FOUND A FACE')
			for (x, y, width, height) in detected_objects:
				cv2.rectangle(self.cv_image, (x, y),
							(x + height, y + width),
							(0, 255, 0), 2)

		cv2.namedWindow('face')
		cv2.imshow('face', self.cv_image)
		cv2.waitKey(3)
		

class colourIdentifier():

	def __init__(self):

		# Initialise any flags that signal a colour has been detected (default to false)
		self.green_found = False
		self.red_found = False

	def start_search(self):
		self.green_found = False
		# Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
		self.bridge = CvBridge()

		# We covered which topic to subscribe to should you wish to receive image data
		self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)

	def start_face_search(self):
		self.red_found = False
		self.bridge = CvBridge()
		self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
		self.desired_velocity = Twist()
		self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback3)
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

		# Set the upper and lower bounds for the colour you wish to identify - green
		self.hsv_green_lower = np.array([25,52,72])
		self.hsv_green_upper = np.array([102,255,255])

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

		#Red Upper and Lower Bounds
		self.hsv_red_lower = np.array([0,100,20])
		self.hsv_red_upper = np.array([5,255,255])

		self.hsv_img = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
		self.mask = cv2.inRange(self.hsv_img,self.hsv_red_lower,self.hsv_red_upper)
		self.result = cv2.bitwise_and(self.cv_image,self.cv_image,mask =self.mask)

		self.contours = cv2.findContours(self.mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[0]


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
						self.pub.publish(self.desired_velocity)
				if cx<300:
					rospy.loginfo("Rotating Left ...")
					self.desired_velocity.angular.z = 0.075
					for i in range (1):
						self.pub.publish(self.desired_velocity)
				self.desired_velocity.angular.z = 0
					
			#If Red object is centered then start moving towards it
			if cv2.contourArea(c) < 15000 and cv2.contourArea(c)>50 and (cx<330 and cx>300):	
				(x, y), radius = cv2.minEnclosingCircle(c)

				cv2.circle(self.result,(int(x),int(y)),int(radius),[155,50,50],5)
				self.desired_velocity.linear.x = 0.3
				for i in range (30):
					self.pub.publish(self.desired_velocity)
			# if (self.red_found == True):
			# 	rospy.loginfo("Red found!")
			if cv2.contourArea(c) > 15000:
				
				rospy.loginfo("Red found!")
				self.red_found = True
				self.desired_velocity.linear.x = 0
				for i in range (30):
					self.pub.publish(self.desired_velocity)
				
		#Show the resultant images you have created. You can show all of them or just the end result if you wish to.
		cv2.namedWindow("dbg_window")
		cv2.imshow("dbg_window",self.cv_image)
		cv2.waitKey(3)

		cv2.namedWindow("window")
		cv2.imshow("window",self.result)

###CLASS FOR THE ROBOT
class Bobot():

	def __init__(self, ents, mids):

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
		self.camera.start_face_search()
		




if __name__ == '__main__':
	try:
		##node
		rospy.init_node('bobot_boy', anonymous=True)

		##NO ARGS = CLEAN RUN; ARGS = DEBUGGING
			##read points from yaml files and do some sorting so coding is easier
		if len(sys.argv) == 1:
			points = []
			with open("./../world/input_points.yaml", 'r') as stream:
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

		###DEBUGGING SECTION, DELETE BEFORE SUBMISSION
		if sys.argv[1] == 'debug_face':


			print('DEBUGGING FACE STUFF')
			facer = faceDetector()
			facer.start_search()
			rospy.spin()
		if sys.argv[1] == 'debug_face_search':


			print('DEBUGGING FACE Search STUFF')
			facerc = colourIdentifier()
			facerc.start_face_search()
			rospy.spin()


	except rospy.ROSInterruptException:
		rospy.loginfo("Ctrl-C caught. Quitting")

	#cv2.destroyWindow('window')


