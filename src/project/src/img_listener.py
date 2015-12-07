#!/usr/bin/python
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

SCALE = 0.33
CRITERIA = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)

def callback(data):
	bridge = CvBridge()
	try:
		big_im = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
		newsize = tuple(int(x*SCALE) for x in big_im.shape[1::-1])
		im = cv2.resize(big_im, dsize=newsize, fx=0, fy=0)

		found, corners = cv2.findChessboardCorners(im, (7,7), None)
		if corners is not None: 
			corners = corners / SCALE
		if found:
			bw = cv2.cvtColor(big_im, cv2.COLOR_BGR2GRAY)
			cv2.cornerSubPix(bw, corners, (5,5), (-1,-1), CRITERIA)
		cv2.drawChessboardCorners(big_im, (7,7), corners, found)
		msg = bridge.cv2_to_imgmsg(big_im, encoding='bgr8')
		pub.publish(msg)
		
	except CvBridgeError, e:
		print(e)


def listener():
	rospy.init_node("image_listener")
	rospy.Subscriber("/cameras/left_hand_camera/image", Image, callback)
	rospy.spin()

if __name__ == "__main__":
	global pub
	pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
	listener()
