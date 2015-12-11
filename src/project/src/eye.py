#! /usr/bin/python
import os, sys, argparse
import numpy as np
import scipy
import cv2 as v

import rospy, tf
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge, CvBridgeError
from baxter_interface import camera as baxter_cam
from project.msg import BoardMessage


# name of the world frame
BASE_FRAME = 'base'


# Side length of chessboard squares, converted from inches to meters.
SQUARE_DIM = 2.25 * 2.54/100
# number of internal corners, not number of squares!
BOARD_SIZE = (7,7)
# inner 7x7 board corners in board-frame coordinates
OBJP = np.zeros((49,3), np.float32)
OBJP[:,:2] = np.mgrid[1:8,1:8].T.reshape(-1,2) * SQUARE_DIM
# outer 4 board corners in board-frame coordinates
OUTER = np.array([[0,0,0],[8,0,0],[0,8,0],[8,8,0]]) * SQUARE_DIM
OUTERH = np.hstack([OUTER, np.ones((4,1))])

# Various parameters for edge refinement algorithm.
CRNR_TERM = (v.TERM_CRITERIA_EPS | v.TERM_CRITERIA_MAX_ITER, 30, 0.001)
CRNR_WIND = (11,11)
CRNR_IGN = (-1,-1)

# the size and shape of the unperspectivified output image
IMSIZE = (256,256)
DSTPTS = np.array([[0,0], [IMSIZE[0],0], [0,IMSIZE[1]], IMSIZE],
                  dtype=np.float32)

# amount to scale by before finding corners
SCALE = 0.33

def find_chessboard_corners(image, scale=1):
    """
    Given an image as a numpy array and a percentage to scale by, tries
    to find a chessboard in the scaled version of the image, then find
    the points to subpixel precision in the original image.
    """
    smaller = v.resize(image, dsize=None, fx=scale, fy=scale)

    found, corners = v.findChessboardCorners(smaller, (7,7), None)

    if corners is not None:
        corners = corners / scale
        if found: v.cornerSubPix(image[:,:,1], corners, CRNR_WIND, 
                                 CRNR_IGN, CRNR_TERM)

    return found, corners

def corner_order(corners, first_axis=1, second_axis=0):
    order = corners[:,first_axis].argsort()
    further_two = corners[order[:2], second_axis].argsort()
    closer_two = corners[order[2:], second_axis].argsort() + 2
    return order[np.hstack([further_two, closer_two])]


def lookup_transform(name):
    return tfl.lookupTransform(BASE_FRAME, name, rospy.Time(0))


def callback(data):
    im = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    # im = v.undistort(im, MTX, DIST)

    found, corners = find_chessboard_corners(im, SCALE)

    if found:
        message = BoardMessage()

        # rotation and translation of board relative to camera
        rvec, tvec, _ = v.solvePnPRansac(OBJP, corners, MTX, DIST)

        # grab & reformat the corners and get the correct order for them
        outer, _ = v.projectPoints(OUTER, rvec, tvec, MTX, DIST)
        outer = np.float32(outer.reshape((4,2)))
        order = corner_order(outer)

        # undistort the image and put it in the message
        M = v.getPerspectiveTransform(outer[order], DSTPTS)
        un = v.warpPerspective(im, M, IMSIZE)
        message.unperspective = bridge.cv2_to_imgmsg(un, encoding='bgr8')
        message.unperspective.header.stamp = rospy.Time.now()

        if PUBLISH_UNDISTORT:
            impub.publish(message.unperspective)

        R, _ = v.Rodrigues(rvec)
        G = np.hstack([R, tvec.reshape((3,1))])
        outer_c = OUTERH[order].dot(G.T)

        print '\n\n================================'
        fields = 'topleft topright botleft botright'.split()
        for i in range(4):
            point = PointStamped()
            point.point = Point()
            point.point.x, point.point.y, point.point.z = outer_c[i]
            point.header = Header()
            point.header.frame_id = 'left_hand_camera'
            point.header.stamp = rospy.Time(0)
            p = tfl.transformPoint(BASE_FRAME, point).point
            setattr(message, fields[i], p)
            print [p.x, p.y, p.z]

        pub.publish(message)

    if PUBLISH_DRAWPOINTS:
        v.drawChessboardCorners(im, (7,7), corners, found)
        ptpub.publish(bridge.cv2_to_imgmsg(im, encoding='bgr8'))



if __name__ == '__main__':
    rospy.init_node('eye')

    desc = 'ROS node that reads the camera, publishes the board TF, ' \
    	'and classifies squares'
    parser = argparse.ArgumentParser(description = desc)
    parser.add_argument('-c', '--calib', required=True,
                        help='camera calibration parameters')
    parser.add_argument('-i', '--image', default='/cameras/left_hand_camera/image',
                        help='input Image topic')
    parser.add_argument('-o', '--out', required=True,
                        help='output BoardMessage topic')
    parser.add_argument('-u', '--undistorted', default=None,
                        help='output undistorted board Image topic')
    parser.add_argument('--boardpoints', default=None,
                        help='output drawChessboardPoints Image topic')
    parser.add_argument('-s', '--scale', type=float, default=SCALE,
                        help='scale this much before corner-finding')
    args = parser.parse_args(rospy.myargv()[1:])
    board_topic = args.out
    in_topic = args.image

    # get the camera calibration parameters
    params = np.load(args.calib)
    MTX, DIST = params['mtx'], params['dist']
    SCALE = args.scale

    # set up the publisher
    pub = rospy.Publisher(board_topic, BoardMessage,
                          latch=True, queue_size=1)
    PUBLISH_UNDISTORT = args.undistorted is not None
    if PUBLISH_UNDISTORT:
        impub = rospy.Publisher(args.undistorted, Image,
                                latch=True, queue_size=1)
    PUBLISH_DRAWPOINTS = args.boardpoints is not None
    if PUBLISH_DRAWPOINTS:
        ptpub = rospy.Publisher(args.boardpoints, Image,
                                latch=True, queue_size=1)

    # set up the CV bridge & TF listener
    bridge = CvBridge()
    tfl = tf.TransformListener()

    # create a camera listener node
    rospy.Subscriber(in_topic, Image, callback)
    print 'Visual cortex ready!'
    rospy.spin()

