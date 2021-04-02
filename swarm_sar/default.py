#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <swarm_sar> environment

Feel free to edit this template as you like!
"""

from morse.builder import *

# Add the MORSE mascott, MORSY.
# Out-the-box available robots are listed here:
# http://www.openrobots.org/morse/doc/stable/components_library.html
#
# 'morse add robot <name> swarm_sar' can help you to build custom robots.

nPath = 1

nDrones = 3
Drones = []
for i in range(1, (nPath+nDrones)+1):
	Robot = Quadrotor("bot"+str(i))
	QRDC = RotorcraftWaypoint()
	pose = Pose()
	IR = VideoCamera()
	cam = VideoCamera()
	IR.properties(cam_width=800, cam_height=600)
	cam.properties(cam_width=800, cam_height=600)
	QRDC.translate(0, 0, 0)
	cam.translate(-0.1,0,-0.2)
	IR.translate(0.1,0,-0.2)
	IR.rotate(0,-1.5702,0)
	cam.rotate(0,-1.5708,0)
	QRDC.add_interface('ros')
	QRDC.add_interface('socket')
	QRDC.properties(MaxBankAngle=0.17)
	pose.add_interface('ros')
	cam.add_interface('ros')
	IR.add_interface('ros')
	if i<=nPath:
		lidar = Sick()
		lidar.properties(Visible_arc = True)
		lidar.properties(resolution = 1.0)
		lidar.properties(scan_window = 360)
		lidar.properties(laser_range = 100.0)
		lidar.properties(layers = 16)
		lidar.properties(layer_separation = 1.0)
		#The docs are kind of unclear on what this does
		lidar.properties(layer_offset = 0.25)
		lidar.translate(0,0,0.25)
		lidar.add_interface('ros')
		Robot.append(lidar)
	Robot.append(QRDC)
	Robot.append(pose)
	Robot.append(cam)
	Robot.append(IR)
	Robot.translate(0,i*2,10)
	Drones.append(Robot)


# set 'fastmode' to True to switch to wireframe mode
env = Environment('mountain/mount.blend1', fastmode = False)
env.set_camera_location([0.0, 0.0, 40])
env.set_camera_rotation([1.09, 0, -1.14])

