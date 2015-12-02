#! /usr/bin/python
import rospy
from sensor_msgs.msg import Image
import argparse
import cv2 as v
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


# Size of the unperspectivified output image
IMSIZE = 512

# Various parameters for edge refinement algorithm.
CRNR_TERM = (v.TERM_CRITERIA_EPS | v.TERM_CRITERIA_MAX_ITER, 30, 0.001)
CRNR_WIND = (5,5)
CRNR_IGN = (-1,-1)


def find_chessboard_corners(image, scale=1):
    """
    Given an already-loaded image and a percentage to scale by, tries to 
    find a chessboard in the scaled version of the image, then find the
    points to subpixel precision in the original image.
    """
    if scale != 1:
        smaller = v.resize(image, dstsize=None, fx=scale, fy=scale)

    found, corners = v.findChessboardCorners(image, (7,7), None)

    if found:
        corners = corners / scale
        v.cornerSubPix(image[:,:,1], corners, CRNR_WIND, CRNR_IGN, CRNR_TERM)

    return found, corners

def get_chessboard_outer_corners(corners, size):
    """
    Given the chessboard inner corners in source image space, and the
    final width and height of the chessboard in output image space,
    computes and returns the outside corners of the board in both
    source and destination image space.
    """
    srccorners = np.zeros((4,2), np.float32)
    dstcorners = np.float32([(0,0), (size,0), (size,size), (0,size)])

    srccorners[0] = 3*corners[0] - corners[7] - corners[1]
    srccorners[1] = 3*corners[6] - corners[5] - corners[13]
    srccorners[2] = 3*corners[48] - corners[47] - corners[41]
    srccorners[3] = 3*corners[42] - corners[43] - corners[35]

    return srccorners, dstcorners


def callback(data):
    im = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

    found, corners = find_chessboard_corners(im)

    if found:
        srccorners, dstcorners = get_chessboard_outer_corners(corners, IMSIZE)

        M = v.getPerspectiveTransform(srccorners, dstcorners)
        un = v.warpPerspective(im, M, (IMSIZE,IMSIZE))

        data = bridge.cv2_to_imgmsg(un, encoding='bgr8')
        pub.publish(data)


if __name__ == '__main__':
    global pub, bridge

    desc = 'ROS node that reads an image, finds a chessboard, and, ' \
           'computes the reverse perspective mapping'
    parser = argparse.ArgumentParser(description=desc)
    parser.add_argument('image_topic')
    parser.add_argument('--name', default='unperspective_node')
    args = parser.parse_args()
    image_topic = args.image_topic
    node_name = args.name

    bridge = CvBridge()

    pub = rospy.Publisher(image_topic + '_unperspective', Image,
                          latch=True, queue_size=1)
    rospy.init_node(node_name)
    rospy.Subscriber(image_topic, Image, callback)

    print 'Ready and unperspectiving!'
    rospy.spin()
