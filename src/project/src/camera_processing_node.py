#! /usr/bin/python
import scipy
import scipy.interpolate
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
OBJP[:,:2] = np.mgrid[-3:4, -3:4].T.reshape(-1,2) * SQUARE_DIM

# Various parameters for edge refinement algorithm.
CRNR_TERM = (v.TERM_CRITERIA_EPS | v.TERM_CRITERIA_MAX_ITER, 30, 0.001)
CRNR_WIND = (5,5)
CRNR_IGN = (-1,-1)

# the size of the unperspectivified output image
IMSIZE = (512,512)
SCALE = 1

bridge = CvBridge()

corners = None
def find_chessboard_corners(image, scale=1):
    global corners
    """
    Given an image as a numpy array and a percentage to scale by, tries
    to find a chessboard in the scaled version of the image, then find
    the points to subpixel precision in the original image.
    Then, fits a multilinear model to those points to avoid dumb outliers.
    """
    smaller = v.resize(image, dsize=None, fx=scale, fy=scale) if scale!=1 else image
    if corners is not None:
        corners = corners * scale

    found, corners = v.findChessboardCorners(image, (7,7), corners)

    if found:
        corners = corners / scale
        v.cornerSubPix(image[:,:,1], corners, CRNR_WIND, 
                       CRNR_IGN, CRNR_TERM)

    return found, corners

def get_chessboard_outer_corners(corners, size):
    """
    Given the chessboard corners in input image space, and the final
    width and height of the chessboard in output image space, computes
    and returns the outside corners of the board in both source and
    destination image space.
    """
    # take the 4 corners outermost of the inner 49
    outer = corners[np.array([0,6,42,48])].copy().reshape((2,2,2))

    # TODO: optimize this interpolation vs. real corners for minimal error
    interp = scipy.interpolate.interpn
    grid = (np.array([0,6]), np.array([0,6]))
    corner_coords = np.array([[-1,-1], [7,-1], [-1,7], [7,7]])
    # xnterp = interp(grid, outer[:,0].reshape((2,2)), corner_coords,
                    # bounds_error=False, fill_value=None)
    # ynterp = interp(grid, outer[:,1].reshape((2,2)), corner_coords,
                    # bounds_error=False, fill_value=None)
    srccorners = interp(grid, outer, corner_coords,
                        bounds_error=False, fill_value=None)
    dstcorners = np.array([(0,0), (size[0],0), (0,size[1]), size], dtype=np.float32)

    # srccorners = np.hstack([xinterp, yinterp])

    # Reorder corners: find closer and further two, then sort each
    #  of those pairs by x. This way, the corners are ordered like:
    #  upper left, upper right, lower left, lower right
    srccorners = srccorners[srccorners[:,1].argsort()]
    further_two = srccorners[:2,0].argsort()
    closer_two = srccorners[2:,0].argsort() + 2
    srccorners = srccorners[np.hstack([further_two, closer_two])]
    return np.float32(srccorners), np.float32(dstcorners)


def callback(data):
    im = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

    if UNDISTORT:
        im = v.undistort(im, MTX, DIST, None, TX) # I don't think these parameters are ordered correctly

    found, corners = find_chessboard_corners(im, SCALE)

    if found:
        print 'Found!'
        srcpts, dstpts = get_chessboard_outer_corners(corners, IMSIZE)
        M = v.getPerspectiveTransform(srcpts, dstpts)
        un = v.warpPerspective(im, M, IMSIZE)

        data = bridge.cv2_to_imgmsg(un, encoding='bgr8')
        impub.publish(data) #I'm assuming this should be *impub* not *pub*

        boardCorners = np.float32([[srcpts[0][0], srcpts[0][1]], [srcpts[1][0], srcpts[1][1]], [srcpts[2][0], srcpts[2][1]], [srcpts[3][0], srcpts[3][1]]])
        #quad_ = cv2.perspectiveTransform(quad.reshape(1, -1, 2), H).reshape(-1, 2) # need to know H matrix
        # boardPoints = # coordinates for board corners in the world frame
        # rvec, tvec = solvePnP(boardPoints, boardCorners, MTX, DIST)
        # trans = TFMessage()
        # trans.transform.translation = tvec
        # trans.transform.rotation = rvec
        # tfpub.publish(trans)
        
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


if __name__ == '__main__':
    global pub, UNDISTORT, MTX, DIST, TX

    desc = 'ROS node that reads the camera, publishes the board TF, ' \
    	'and classifies squares'
    parser = argparse.ArgumentParser(description = desc)
    parser.add_argument('-c', '--calib',
                        help='camera calibration parameters')
    parser.add_argument('-i', '--image', required=True,
                        help='input image topic')
    parser.add_argument('-o', '--out', required=True,
                        help='output image topic')
    parser.add_argument('--tf', required=True,
                        help='TF output topic')
    parser.add_argument('-s', '--scale', type=float, default=SCALE,
                        help='scale this much before corner-finding')
    args = parser.parse_args()
    out_topic = args.out
    tf_out_topic = args.tf
    in_topic = args.image

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

