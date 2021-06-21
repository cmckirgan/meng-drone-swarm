import rospy
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from sensor_msgs.msg import LaserScan, Image, PointCloud2
import transforms as tf
import math
import numpy as np
import cv2
import sensing
import logging
import laser_geometry.laser_geometry as lg
#Search modules defines the robots and their control logic

class robot:
#Base robot class, used for commonality between the pathfinders and the 
#slave drones

	def __init__(self, Rank):
		#TODO: slow the speed of the drone, helps with camera issues
		#Kinda done makes them overshoot like mad
		print("Creating Done")
		self.pose = rospy.Subscriber("/bot"+str(Rank)+"/pose", PoseStamped, self.pose_ctrl)
		self.global_pose = rospy.Subscriber("global_pose", PoseArray, self.global_pose_ctrl)
		self.QRDC = rospy.Publisher("/bot"+str(Rank)+"/QRDC", Pose, queue_size=10)
		self.cam = rospy.Subscriber("/bot"+str(Rank)+"/cam_r/image", Image, self.cam_handler)
		self.IR = rospy.Subscriber("/bot"+str(Rank)+"/cam_l/image", Image, self.IR_handler)
		#rospy.init_node("Robot"+str(Rank))
		self.rank = Rank
		self.waypoint = []
		self.current_dests = []
		self.hold = True
		self.search = True
		self.position = PoseStamped()
		self.PosLoc = []

	def global_pose_ctrl(self, message):
		#Recives a message containg the current postion and destination of all 
		#drones. Use to calculate collions and ajust flightpath acordingly
		#print(message)
		self.check_collisions(message)

	def set_waypoints(self, waypoint):
		#TODO: rotate the robot to face the direction of the way point, fixes some camera issues
		self.waypoint.append(waypoint)
		print(self.waypoint)

	def pose_ctrl(self, message):
		self.position = message
		try:
			if self.waypoint:
				if (abs(message.pose.position.x - self.waypoint[0].position.x)<= 0.5) and (abs(message.pose.position.y-self.waypoint[0].position.y) <= 0.5) and (abs(message.pose.position.z -self.waypoint[0].position.z) <= 0.5):
					print("Postion Reached "+str(self.rank))
					self.waypoint.pop(0)
					#print(self.waypoint[0])
				else:
					if len(self.waypoint)>0:
						if self.hold:
							pass
#							This publishes posestammped not pose
#							self.QRDC.publish(self.position)
						else:
							#TODO produce quaternion for rotation to that point
#							print("Reached")
							tmp_waypoint = self.waypoint[0]
							rotation_qua = tf.euler_to_quaternion(0,0,math.atan2(self.position.pose.position.x-tmp_waypoint.position.x, self.position.pose.position.y-tmp_waypoint.position.y))
							tmp_waypoint.orientation.x = rotation_qua[0]
							tmp_waypoint.orientation.y = rotation_qua[1]
							tmp_waypoint.orientation.z = rotation_qua[2]
							tmp_waypoint.orientation.w = rotation_qua[3]
#							print("Rotation: "+str(rotation_qua))
							self.QRDC.publish(tmp_waypoint)
			else:
#				print("No more Waypoints")
				self.hold = True
				if self.search:
		#			print(self.PosLoc)
					self.search = False

		except Exception as e:
			print(e)
			self.hold = True

	def check_collisions(self, message):
#		ego_vert = [(self.position.x+0.5, self.positon.y+0.5, self.position.z+0.5),
#			(self.position.x-0.5, self.position.y+0.5, self.position.z+0.5),
#			(self.position.x+0.5, self.position.y+0.5, self.position.z-0.5),
#			(self.position.x-0.5, self.position.y+0.5, self.position.z-0.5),
#			(self.waypoint[0].position.x+0.5, self.waypoint[0].position.y-0.5, self.waypoint[0].z+0.5),
#			(self.waypoint[0].position.x-0.5, self.waypoint[0].position.y-0.5, self.waypoint[0].z+0.5),
#			(self.waypoint[0].position.x+0.5, self.waypoint[0].position.y-0.5, self.waypoint[0].z-0.5),
#			(self.waypoint[0].position.x-0.5, self.waypoint[0].position.y-0.5, self.waypoint[0].z-0.5)]
		for i in range(1, len(message.poses)):
			#Test points intersect and alter course
			#Implentation of Sutherland-Hodgman Algo
				#Create other vert
				#Check for polygon clipping, if clipping hold lower ranked drone
			if i < self.rank and self.calc_dist(self.position.pose, message.poses[i-1]) <=10:
				self.hold = True
				break
			else:
				self.hold = False
#		print("Robot "+str(self.rank)+" holding!")

	def calc_dist(self, ego, target):
		return math.sqrt(((ego.position.x - target.position.x)**2) + ((ego.position.y - target.position.y)**2) + ((ego.position.z - target.position.z)**2))

	def cam_handler(self, message):
		self.cam_img = message.data

	def IR_handler(self, message):
#		print("IR Recivied")
		self.IR_img = message
		detection = sensing.detectHeat(self.IR_img)
		if (detection is not None) and self.search:
#			print("Heat found by bot %d" % (self.rank))
			detection = sensing.detectHuman(self.IR_img, self.position)
			if detection is not None:
				#self.PosLoc.append(list(detection[0]))
				if not self.PosLoc:
					self.PosLoc.append(list(detection[0]))
					print('First Body found by bot:%d at X:%d Y%d' % (self.rank, detection[0][0], detection[0][1]))
				else:
					newBody = False
					for i in range(0,len(self.PosLoc)):
						#Establishes a 20m region of error in which new bodies will be ignored
						if (abs(self.PosLoc[0][0]-detection[0][0]) > 20.0) and (abs(self.PosLoc[0][1]-detection[0][1]) > 20.0):
							newBody = True
						else:
							pass
					if newBody:
						self.PosLoc.append(list(detection[0]))
						print('New Body found by bot:%d at X:%d Y:%d' % (self.rank, detection[0][0], detection[0][1]))
			else:
				pass
#				print('No body')
		#Run blob detction
		#Publish image and pose to controller
		#Maybe classifer if computaion allows

class pathfinder(robot):
#Extends robot class, used for definition of the master pathfinder drones
	lidar = None
	lidar_data = [[],[]]
	def __init__(self,Rank):
		super().__init__(Rank)
		self.lidar = rospy.Subscriber("/bot"+str(Rank)+"/lidar", LaserScan, self.lidar_handler)
		self.slam_node = rospy.Publisher("/velodyne_points", PointCloud2, queue_size=10)
		self.lp = lg.LaserProjection()

	def lidar_handler(self, message):
		#If mapping has begun convert and publish lidar scan to slam node
		if not self.search:
			ls2pc = self.lp.projectLaser(message)
			self.slam_node.publish(ls2pc)
			#Once mapping is complete point cloud map can be viewed in RVIS or saved as a pcm file.

class drone(robot):
#Extends robot class, used for definition of the slave drones
	def drone(Rank):
		super().__init__(Rank)


class drone_control():
	drones = []
	waypoints = [[],[]]
	POI = []

	def __init__(self, nPath, nDrones, Radius, Theta, Alt):
		rospy.init_node("Control")
		self.create_drones(nDrones, nPath)
		self.calculate_init_waypoints(Radius, Theta, Alt)
		self.mapping = False
		#Might change this to publish as a 2D array
		#In real world applications these would be published by each drone
		self.global_pose_pub = rospy.Publisher('global_pose', PoseArray, queue_size=10)
		self.global_dest_pub = rospy.Publisher('global_dest', PoseArray, queue_size=10)
		#rospy.init_node("control")
		#rospy.spin()

	def start(self):
		#Publish inital destinations to drones
		count = 0
		for i in range(0, len(self.drones)):
			#Publish start and end to each drone or until all inital waypoints are published
			if count < len(self.waypoints[0]):
				self.drones[i].set_waypoints(self.waypoints[0][count])
				self.drones[i].set_waypoints(self.waypoints[1][count])
				count += 1
			else:
				break

		self.rate= rospy.Rate(10)
		while not rospy.is_shutdown():
			self.rate.sleep()
			global_pose = PoseArray()
			global_dest = PoseArray()

			for i in range(0, len(self.drones)):
				#Update the postion and destination of each drone for collison avoidance
				pose = self.drones[i].position.pose
				if len(self.drones[i].waypoint) > 0:
					dest = self.drones[i].waypoint[0]
				else:
					#Done has finished it's search path
					if len(self.waypoints[0])>count:
					#If there are more waypoints to be searched assaign them to drones
						self.drones[i].set_waypoints(self.waypoints[0][count])
						self.drones[i].set_waypoints(self.waypoints[1][count])
						dest = self.drones[i].waypoint[0]
						count += 1
					elif len(self.POI)>0:
#						print("Default Path Completed")
						dest = pose
					else:
						dest = pose
				global_pose.poses.append(pose)
				global_dest.poses.append(dest)

			self.global_pose_pub.publish(global_pose)
			self.global_dest_pub.publish(global_dest)
			searchComplete = True
			for i in range(0, len(self.drones)):
				#Check all drones have finished searching
				searchComplete = searchComplete and not self.drones[i].search
			if searchComplete and not self.mapping:
				#Find the avgerage postion of the bodies and begin mapping
				print('Search Phase Complete')
				searchLoc = []
				for drone in self.drones:
				#Clean up locations gathered by each drone, produced a list of all unique points
					posList = [list(i) for i in set(tuple(i) for i in drone.PosLoc)]
					for j in posList:
						searchLoc.append(j)
				x_avg = 0
				y_avg= 0
				for loc in searchLoc:
					x_avg += loc[0]
					y_avg += loc[1]
				x_avg = x_avg/len(searchLoc)
				y_avg = y_avg/len(searchLoc)
				print('Search Location is X:%d, Y:%d' % (x_avg, y_avg))

				mapping_waypoints = self.calc_mapping_waypoints(x_avg,y_avg)
				#self.drones[0].waypoints = mapping_waypoints
				for point in mapping_waypoints:
					self.drones[0].set_waypoints(point)
				self.mapping = True

	def create_drones(self, nDrones, nPath):
	#Creates the correct number of pathfinder and slave drones, then adds them to the array of
	#drones.
	#nDrones - Number of slave drones
	#nPath - Number of pathfinder drones
		for i in range(1, (nDrones+nPath)+1):
			if i <= nPath:
				self.drones.append(pathfinder(i))
			else:
				self.drones.append(drone(i))

	def calculate_init_waypoints(self, radius, theta, Alt):
		for i in range(0, int((360/theta)/2)):
			#Calculate lines of len r*2
			angle = math.radians(i*theta)
			start_x = radius*math.sin(angle)
			start_y = radius*math.cos(angle)
			start_pose = Pose()
			start_pose.position.x = start_x
			start_pose.position.y = start_y
			start_pose.position.z = Alt
			end_pose = Pose()
			end_pose.position.x = -start_x
			end_pose.position.y = -start_y
			end_pose.position.z = Alt
			print(end_pose.orientation)
			self.waypoints[0].append(start_pose)
			self.waypoints[1].append(end_pose)
		#print(self.waypoints)
	def calc_mapping_waypoints(self, x, y):
		waypoints = []
		points = [(x,y+20), (x+20,y),(x,y-20),(x-20,y)]
		for point in points:
			pose1 = Pose()
			pose2 = Pose()
			pose3 = Pose()
			pose1.position.x = point[0]
			pose2.position.x = point[0]
			pose3.position.x = point[0]
			pose1.position.y = point[1]
			pose2.position.y = point[1]
			pose3.position.y = point[1]
			pose1.position.z = 70.0
			pose2.position.z = 20.0
			pose3.position.z = 70.0
			waypoints.append(pose1)
			waypoints.append(pose2)
			waypoints.append(pose3)
		return waypoints

controller = drone_control(1, 3, 20, 30, 75)
controller.start()

#def callback_1(msg):
 #   print("test")
  #  waypoint_bot1.position.x = waypoints[0][0][0]
   # waypoint_bot1.position.y = waypoints[0][0][1]
    #cmd_bot1.publish(waypoint_bot1)
    #init = True


#rospy.init_node("search1")
#rospy.Subscriber("/bot1/pose1", PoseStamped, callback_1)
#rospy.spin()
