#! /usr/bin/python
import os, sys, argparse
import numpy as np
import cv2 as v
import rospy
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge, CvBridgeError
from baxter_interface import camera as baxter_cam

# Side length of chessboard squares, converted from inches to meters.
SQUARE_DIM = 2.25 * 2.54/100
# number of internal corners, not number of squares
BOARD_SIZE = (7,7)
# real-world size & coordinates of the board corners
OBJP = np.zeros((49,3), np.float32)
OBJP[:,:,2] = np.mgrid[-3:4, -3:4].T.reshape(-1,2) * SQUARE_DIM

# Various parameters for edge refinement algorithm.
CRNR_TERM = (v.TERM_CRITERIA_EPS | v.TERM_CRITERIA_MAX_ITER, 30, 0.001)
CRNR_WIND = (5,5)
CRNR_IGN = (-1,-1)

# the size of the unperspectivified output image
IMSIZE = (512,512)


bridge = CvBridge()


def find_chessboard_corners(image, scale=1):
    """
    Given an image as a numpy array and a percentage to scale by, tries
    to find a chessboard in the scaled version of the image, then find
    the points to subpixel precision in the original image.
    Then, fits a multilinear model to those points to avoid dumb outliers.
    """
    if scale != 1:
        smaller = v.resize(image, dstsize=None, fx=scale, fy=scale)

    found, corners = v.findChessboardCorners(image, (7,7), None)

    if found:
        corners = corners / scale
        v.cornerSubPix(image[:,:,1], corners, CRNR_WIND, 
                       CRNR_IGN, CRNR_TERM)

        # TODO: fit a linear model to these corners to minimize error

    return found, corners

def get_chessboard_outer_corners(corners, size):
    """
    Given the chessboard inner corners in source image space, and the
    final width and height of the chessboard in output image space,
    computes and returns the outside corners of the board in both
    source and destination image space.
    """
    srccorners = np.zeros((4,2), np.float32)
    dstcorners = np.float32([(0,0), (size[0],0), size, (0,size[1])])

    srccorners[0] = 3*corners[0] - corners[7] - corners[1]
    srccorners[1] = 3*corners[6] - corners[5] - corners[13]
    srccorners[2] = 3*corners[48] - corners[47] - corners[41]
    srccorners[3] = 3*corners[42] - corners[43] - corners[35]

    # TODO: reorder the outside corners so no flipping happens.
    #  Should assume that the board is more or less square to 
    #   the sides of the image; the movement node will choose 
    #   its hand position based on trying to see the board as
    #   well as possible.

    return srccorners, dstcorners


def callback(data):
    im = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    if UNDISTORT:
        im = v.undistort(im, MTX, DIST, None, TX)

    found, corners = find_chessboard_corners(im, SCALE)

    if found:
        srcpts, dstpts  = get_chessboard_outer_corners(corners, IMSIZE)
        M = v.getPerspectiveTransform(srcpts, dstpts)
        un = v.warpPerspective(im, M, IMSIZE)

        data = bridge.cv2_to_imgmsg(un, encoding='passthrough')
        pub.publish(data)

        # TODO: find the TF of the board using v.solvePnP(), 
        #   and publish it as a TFMessage.
        # See http://docs.opencv.org/2.4/modules/calib3d/doc/ ...
        #  ... camera_calibration_and_3d_reconstruction.html#solvepnp


def setup_camera():
    c = baxter_cam.CameraController('left_hand_camera')
    c.resolution = RESOLUTION
    c.exposure = EXPOSURE
    c.gain = GAIN
    c.white_balance_red = BALANCE_RGB[0]
    c.white_balance_green = BALANCE_RGB[1]
    c.white_balance_blue = BALANCE_RGB[2]


if __name__ == '__main__'():
    global pub, UNDISTORT, MTX, DIST, TX

	desc = 'ROS node that reads the camera, publishes the board TF, ' \
		'and classifies squares'
	parser = argparse.ArgumentParser(description = desc)
    parser.add_argument('-c', '--calib',
                        help='camera calibration parameters')
    parser.add_argument('-i', '--in', required=True,
                        help='input image topic')
    parser.add_argument('-o', '--out', required=True,
                        help='output image topic')
    parser.add_argument('--tf', required=True,
                        help='TF output topic')
    parser.add_argument('-s', '--scale', type=float, default=SCALE
                        help='scale this much before corner-finding')
    args = parser.parse_args()

    # get the camera calibration parameters
    if args.calib is not None:
        UNDISTORT = True
        params = np.load(args.calib)
        MTX, DIST = params['mtx'], params['dist']
        TX, _ = v.getOptimalNewCameraMatrix(mtx, dist, RESOLUTION, 
                                            1, RESOLUTION)
    else: UNDISTORT = False

    SCALE = args.scale

    # set up the publisher
    impub = rospy.Publisher(out_topic, Image, 
                            latch=True, queue_size=1)
    tfpub = rospy.Publisher(tf_out_topic, TFMessage, 
                            latch=True, queue_size=1)

    # create a camera listener node
    rospy.init_node('camera_listener')
    rospy.Subscriber(in_topic, Image, callback)
    rospy.spin()

