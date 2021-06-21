import math
import numpy as np
def qua2euler(x, y, z, w):
	"""
	Converts a quaterion into euler angles (roll, pitch, yaw)
	"""
	t0 = 2.0*(w*x+y*z)
	t1 = 1.0-2.0*(x*x+y*y)
	roll_x = math.atan2(t0, t1)

	t2 = 2.0*(w*y-z*x)
	t2 = 1.0 if t2 > 1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	pitch_y = math.asin(t2)

	t3 = 2.0*(w*z+x*y)
	t4 = 1.0 - 2.0*(y*y+z*z)
	yaw_z = math.atan2(t3,t4)

	return roll_x, pitch_y, yaw_z

def euler_to_quaternion(roll, pitch, yaw):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]


def qua_between_points(q0, q1):
	q0 = np.array(q0)
	q1 = np.array(q1)
	return q0*q1

def polar_to_cart(r,t):
	x = r*math.cos(t)
	y = r*math.sin(t)
	return x, y


def cart_to_polar(x, y):
	r = math.sqrt((y**2)+(x**2))
	t = math.atan2(y,x)
	#Returns polar coordinates
#	print(t)
	if (x>= 0 and y >=0):
		return r,t
	elif(x<0 and y>=0):
#		t = math.pi-t
		return r, t
	elif(x<0 and y<0):
#		t = t +math.pi
		return r, t
	elif (x>=0 and y<0):
#		print('Quad 4 %f' % t)
#		t = (math.pi*2)-t
		return r, t


def rotate_polar(r, t, rotation):
	if t < 0:
		t = (math.pi*2)+t
	t = t+rotation
	if t > (math.pi*2):
		t = t -(math.pi*2)
	return r, t
