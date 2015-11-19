#!/usr/bin/env/ python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

def callback(data):
	bridge = CvBridge()
	try:
	    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")	
	    cv2.imshow("Image window", cv_image)
    	    cv2.waitKey(3)
		
	except CvBridgeError, e:
	    print(e)


def listener():
	rospy.init_node("image_listener")
	rospy.Subscriber("/cameras/right_hand_camera/image", Image, callback)
	rospy.spin()

if __name__ == "__main__":
	listener()
