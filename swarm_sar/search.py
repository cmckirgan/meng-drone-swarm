import rospy
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from sensor_msgs.msg import LaserScan, Image
import math
import cv2

class robot:
#Base robot class, used for commonality between the pathfinders and the 
#slave drones

	def __init__(self, Rank):
		#TODO: slow the speed of the drone, helps with camera issues
		print("Creating Done")
		self.pose = rospy.Subscriber("/bot"+str(Rank)+"/pose", PoseStamped, self.pose_ctrl)
		self.global_pose = rospy.Subscriber("global_pose", PoseArray, self.global_pose_ctrl)
		self.QRDC = rospy.Publisher("/bot"+str(Rank)+"/QRDC", Pose, queue_size=10)
		self.cam = rospy.Subscriber("/bot"+str(Rank)+"/cam", Image, self.cam_handler)
		self.IR = rospy.Subscriber("/bot"+str(Rank)+"/IR", Image, self.IR_handler)
		#rospy.init_node("Robot"+str(Rank))
		self.rank = Rank
		self.waypoint = []
		self.current_dests = []
		self.hold = True
		self.position = PoseStamped()

	def global_pose_ctrl(self, message):
		#Recives a message containg the current postion and destination of all 
		#drones. Use to calculate collions and ajust flightpath acordingly
		#print(message)
		self.check_collisions(message)

	def set_waypoints(self, waypoint):
		#TODO: rotate the robot to face the direction of the way point, fixes some camera issues
		self.waypoint.append(waypoint)
		#print(self.waypoint)

	def pose_ctrl(self, message):
		self.position = message
		try:
			if (abs(message.pose.position.x - self.waypoint[0].position.x)<= 0.5) and (abs(message.pose.position.y-self.waypoint[0].position.y) <= 0.5):
				print("Postion Reached "+str(self.rank))
				self.waypoint.pop(0)
				#print(self.waypoint[0])
			else:
				if len(self.waypoint)>0:
					if self.hold:
						self.QRDC.publish(self.position)
					else:
						self.QRDC.publish(self.waypoint[0])
		except:
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
			if i < self.rank and self.calc_dist(self.position.pose, message.poses[i-1]) <=5:
				self.hold = True
				break
			else:
				self.hold = False
		print("Robot "+str(self.rank)+" holding!")

	def calc_dist(self, ego, target):
		return math.sqrt(((ego.position.x - target.position.x)**2) + ((ego.position.y - target.position.y)**2) + ((ego.position.z - target.position.z)**2))

	def cam_handler(self, message):
		self.cam_img = message.image

	def IR_handler(self):
		self.IR_img = message

class pathfinder(robot):
#Extends robot class, used for definition of the master pathfinder drones
	lidar = None
	lidar_data = [[],[]]
	def __init__(self,Rank):
		super().__init__(Rank)
		self.lidar = rospy.Subscriber("/bot"+str(Rank)+"/lidar", LaserScan, self.lidar_handler)

	def lidar_handler(self, message):
		self.lidar_data = message

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

		#Might change this to publish as a 2D array
		self.global_pose_pub = rospy.Publisher('global_pose', PoseArray, queue_size=10)
		self.global_dest_pub = rospy.Publisher('global_dest', PoseArray, queue_size=10)
		#rospy.init_node("control")
		#rospy.spin()

	def start(self):
		#Publish inital destinations to drones
		count = 0
		for i in range(0, len(self.drones)):
			#Publish start and end to each drone or until all inital waypoints are published
			print(count)
			if count < len(self.waypoints[0]):
				self.drones[i].set_waypoints(self.waypoints[0][count])
#				print(self.waypoints[0][count])
				self.drones[i].set_waypoints(self.waypoints[1][count])
				#print("Drone "+str(i)+" Current Dest:")
				#print(self.drones[i].waypoint[0])
				count += 1
			else:
				break

		self.rate= rospy.Rate(10)
		while not rospy.is_shutdown():
			self.rate.sleep()
			global_pose = PoseArray()
			global_dest = PoseArray()

			for i in range(0, len(self.drones)):
				pose = self.drones[i].position.pose
				if len(self.drones[i].waypoint) > 0:
					dest = self.drones[i].waypoint[0]
				else:
					#Done has finished it's search path
					if len(self.waypoints[0])>count:
						self.drones[i].set_waypoints(self.waypoints[0][count])
						self.drones[i].set_waypoints(self.waypoints[1][count])
						dest = self.drones[i].waypoint[0]
						count += 1
					elif len(self.POI)>0:
						print("Default Path Completed")
						dest = pose
					else:
						dest = pose
				global_pose.poses.append(pose)
				global_dest.poses.append(dest)
			self.global_pose_pub.publish(global_pose)
			self.global_dest_pub.publish(global_dest)


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
			self.waypoints[0].append(start_pose)
			self.waypoints[1].append(end_pose)
		#print(self.waypoints)


controller = drone_control(1, 3, 20, 30, 25)
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
