import cv2
import transforms
import numpy as np
from cv_bridge import CvBridge
#Sensing Module is used to detect and classify objects found by the drone swarm.


name_to_dtypes = {
	"rgb8":    (np.uint8,  3),
	"rgba8":   (np.uint8,  4),
	"rgb16":   (np.uint16, 3),
	"rgba16":  (np.uint16, 4),
	"bgr8":    (np.uint8,  3),
	"bgra8":   (np.uint8,  4),
	"bgr16":   (np.uint16, 3),
	"bgra16":  (np.uint16, 4),
	"mono8":   (np.uint8,  1),
	"mono16":  (np.uint16, 1),

    # for bayer image (based on cv_bridge.cpp)
	"bayer_rggb8":	(np.uint8,  1),
	"bayer_bggr8":	(np.uint8,  1),
	"bayer_gbrg8":	(np.uint8,  1),
	"bayer_grbg8":	(np.uint8,  1),
	"bayer_rggb16":	(np.uint16, 1),
	"bayer_bggr16":	(np.uint16, 1),
	"bayer_gbrg16":	(np.uint16, 1),
	"bayer_grbg16":	(np.uint16, 1),

    # OpenCV CvMat types
	"8UC1":    (np.uint8,   1),
	"8UC2":    (np.uint8,   2),
	"8UC3":    (np.uint8,   3),
	"8UC4":    (np.uint8,   4),
	"8SC1":    (np.int8,    1),
	"8SC2":    (np.int8,    2),
	"8SC3":    (np.int8,    3),
	"8SC4":    (np.int8,    4),
	"16UC1":   (np.uint16,   1),
	"16UC2":   (np.uint16,   2),
	"16UC3":   (np.uint16,   3),
	"16UC4":   (np.uint16,   4),
	"16SC1":   (np.int16,  1),
	"16SC2":   (np.int16,  2),
	"16SC3":   (np.int16,  3),
	"16SC4":   (np.int16,  4),
	"32SC1":   (np.int32,   1),
	"32SC2":   (np.int32,   2),
	"32SC3":   (np.int32,   3),
	"32SC4":   (np.int32,   4),
	"32FC1":   (np.float32, 1),
	"32FC2":   (np.float32, 2),
	"32FC3":   (np.float32, 3),
	"32FC4":   (np.float32, 4),
	"64FC1":   (np.float64, 1),
	"64FC2":   (np.float64, 2),
	"64FC3":   (np.float64, 3),
	"64FC4":   (np.float64, 4)
}

def imgMsg2cv2(msg, encoding):
	#Converts ROS img msg to a cv2 image
	dtype_class, n_channels = name_to_dtypes[msg.encoding]
	dtype = np.dtype(dtype_class)
	dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')

	if n_channels == 1:
		im = ndarray(shape =(msg.height, msg.width), dtype = dtype, buffer=msg.data)
	else:
		if(type(msg.data) == str):
			im = np.ndarray(shape=(msg.height, msg.width, n_channels),
			 dtype = dtype, buffer=msg.data.encode())
		else:
			im = np.ndarray(shape = (msg.height, msg.width, n_channels),
			 dtype = dtype, buffer = msg.data)

	if encoding == 'passthrough':
		return im

def detectHeat(img_msg):
	#Detected hot objects in thermal images
	#Grayscles the image
	#Runs a mask for bright pixels
	#Returns the number of hot objects and their position in the image
#	bridge = CvBridge()
#	img = bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
	img = imgMsg2cv2(img_msg, 'passthrough')
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	mask = cv2.inRange(gray, 200, 255)

	x, y, w, h = cv2.boundingRect(mask)

	if not (x == 0 and y == 0 and w == 0 and h == 0):
		bodies =[]
		if type(x) is list:
			for i in range(0, len(x)):
				bodies.append(((x+(x+w))/2.0, (y+(y+h))/2.0))
			return bodies
		else:
			bodies.append(((x+(x+w))/2.0, (y+(y+h))/2.0))
			return bodies
	else:
		return None

def detectHuman(img, pose):
	#Takes an image with hot objects
	#Runs a haar classifier to detect human body shape in those images
	#Returns the number of bodies and position in the image
	classifier = cv2.CascadeClassifier('IRcascade2.xml')

	#Depending on runtime img may need cropped
	img = imgMsg2cv2(img, 'passthrough')
	bodies = classifier.detectMultiScale(img, 1.03, 3)

	detection = []
	position = []
	count = 0

	if isinstance(bodies, np.ndarray):
		for x, y, w, h in bodies:
			detection.append(((x+(x+w))/2.0, (y+(y+h))/2.0))
#			count += 1
			pixel_length = ((x+w)-x) if ((x+w)-x) > ((y+h)-y) else ((y+h)-y)
			#Avg Human length roughly 165cm, 160 chosen to account for curling
			pixel_size = 1.60/pixel_length
			avg_x = (x+(x+h))/2.0
			avg_y = (y+(y+h))/2.0
			cart = ((avg_x-300)*pixel_size,(400-avg_y)*pixel_size)
#			print('Images before rotation X:%f, Y:%f' % (cart[0], cart[1]))
			pitch, roll, yaw = transforms.qua2euler(pose.pose.orientation.x, pose.pose.orientation.y,
			pose.pose.orientation.z, pose.pose.orientation.w)
#			print('Yaw %f' % yaw)
			r,t = transforms.cart_to_polar(cart[0], cart[1])
			r,t = transforms.rotate_polar(r,t,yaw)
			x,y = transforms.polar_to_cart(r, t)
			#Use pixel size to find distance from centre
			position.append((int(pose.pose.position.x+x),int(pose.pose.position.y+y)))
#			print('Body at X:%d Y:%d' %(detection[count-1][0],detection[count-1][1]))
			#Convert to polar, apply roataion of drone about y-axis, convert back to cart and add to 
			#Robot position
		return position
	else:
#		print(type(bodies))
		return None
