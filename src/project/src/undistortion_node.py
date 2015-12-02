#! /usr/bin/python
import rospy
from sensor_msgs.msg import Image
import argparse
import cv2 as v
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from baxter_interface import camera as baxter_cam

# the parameters to be used in camera setup
RESOLUTION = (1280, 800)
BALANCE_RGB = (-1,-1,-1)
EXPOSURE = 45
GAIN = 45


def callback(data):
    im = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

    ur = v.undistort(im, MTX, DIST, None, TX)

    data_ur = bridge.cv2_to_imgmsg(ur, encoding='bgr8')
    pub.publish(data_ur)
    

def setup_camera():
    c = baxter_cam.CameraController('left_hand_camera')
    c.resolution = RESOLUTION
    c.exposure = EXPOSURE
    c.gain = GAIN
    c.white_balance_red, c.white_balance_green, c.white_balance_blue = BALANCE_RGB        

if __name__ == '__main__':
    global pub, bridge, TX

    desc = 'ROS node that reads the camera, publishes the board TF, ' \
        'and classifies squares'
    parser = argparse.ArgumentParser(description = desc)
    parser.add_argument('-c', '--calib', required=True,
                        help='camera calibration parameters')
    parser.add_argument('image_topic')
    args = parser.parse_args()
    image_topic = args.image_topic

    # get the camera calibration parameters
    params = np.load(args.calib)
    MTX, DIST = params['mtx'], params['dist']
    TX, _ = v.getOptimalNewCameraMatrix(MTX, DIST, RESOLUTION, 1, RESOLUTION)


    setup_camera()
    bridge = CvBridge()

    pub = rospy.Publisher(image_topic + '_undistorted', Image, 
                          latch=True, queue_size=1)
    rospy.init_node('undistortion_node')
    rospy.Subscriber(image_topic, Image, callback)

    print 'Ready and converting!'
    rospy.spin()


