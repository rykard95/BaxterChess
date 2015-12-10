#! /usr/bin/python
import os
import numpy as np
import cv2 as v
import argparse
from scipy.misc import imsave

def load_camera_calibration(calib):
    """
    Load camera calibration parameters MTX and DIST from the file CALIB.
    """
    params = np.load(calib)
    return params['mtx'], params['dist']



def undistorted_imread(mtx, dist, board):
    """
    Given camera distortion coefficients MTX and DIST, load an image from
    the file BOARD, then undistort and return it.
    """
    # load the board image
    img = v.imread(board)
    wh = img.shape[1::-1]
    tx, roi = v.getOptimalNewCameraMatrix(mtx, dist, wh, 1, wh)

    # undo the camera distortion
    return v.undistort(img, mtx, dist, None, tx)



def find_chessboard_corners(image, scale=None):
    """
    Given an already-loaded image and a percentage to scale by, tries to 
    find a chessboard in the scaled version of the image, then find the
    points to subpixel precision in the original image.
    """
    if scale is not None and scale != 1:
        smaller = v.resize(image, dstsize=None, fx=scale, fy=scale)

    found, corners = v.findChessboardCorners(image, (7,7), None)



# no board: 4, 5, 20, 21
def evaluate_square(arr):
    # WITH STD OF JUST GREEN CHANNEL
    # 3, but that's hard anyway
    # 12, 13, 22, 23 fails because one white square is washed out
    # 38 fails because white bled into black
    # 18, 19 misclassifies more than one!

    # WITH STD OF BW IMAGE
    # 2 3
    arr = v.cvtColor(arr, v.COLOR_BGR2GRAY)
    return np.std(arr)




def getChessboardOutsideCorners(corners, size):
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



if __name__ == '__main__':

    # read command line
    parser = argparse.ArgumentParser()
    parser.add_argument('calib', help='calibration parameters file to use')
    parser.add_argument('filenames', metavar='IMAGE', nargs='+',
                        help='board image file')
    parser.add_argument('-v', action='store_true')
    parser.add_argument('-w', '--write', action='store_true',
                        help='create output image file for each square')
    parser.add_argument('number', type=int, help='enter the starting number')
    args = parser.parse_args()
    calib = args.calib
    filenames = args.filenames
    WRITE = args.write
    VERBOSE = args.v
    num = args.number

    # load the image and undistort it
    mtx, dist = load_camera_calibration(calib)
    for board in filenames:
        print(board)

        img = undistorted_imread(mtx, dist, board)
        bw = img # v.cvtColor(img, v.COLOR_RGB2GRAY)

        # find the chessboard in the undistorted image
        found, corners = v.findChessboardCorners(bw, (7,7), None)

        if found:
            # sharpen the corners
            crit = (v.TERM_CRITERIA_EPS | v.TERM_CRITERIA_MAX_ITER, 30, 0.1)
            # v.cornerSubPix(bw, corners, (11,11), (-1,-1), crit)

            # find the perspective transform
            size = 256
            pts1, pts2 = getChessboardOutsideCorners(corners, size)
            M = v.getPerspectiveTransform(pts1, pts2)
            unwarped = v.warpPerspective(bw, M, (size,size))

            # show the unwarped image
            # v.imshow('test', unwarped)

        # draw the corners and display the image
        if VERBOSE:
            v.drawChessboardCorners(img, (7,7), corners, found)
            v.imshow('old', img)
            v.waitKey(0)

        if not found: 
            print('Argh, no can find!')
            continue

        # a list of all the chessboard squares; should be in order from
        # top left to bottom right just like the labels
        squares = []
        # the width of each square in unwarped
        delta = size/8
        # the number of border pixels on each side to drop out; I used 8
        #  earlier to ignore the border, but I think 0 makes sense for a 
        #  classifier with a little more brains than just np.std
        ign = 0
        for i in range(8):
            y = i*delta
            for j in range(8):
                x = j*delta
                squares.append(unwarped[x+ign:x+delta-ign, y+ign:y+delta-ign])
        
        # squares.sort(key=evaluate_square)
        for square in squares:
            imsave(str(num) + ".jpg", square)
            num += 1

        # Create and display a single 8x8 board image of just the squares 
        squaresprime = []
        #for i in range(8):
        #    squaresprime.append(np.hstack(squares[i*8:(i+1)*8]))
        #out = np.vstack(squaresprime)
        #v.imshow('out', out)
        #v.waitKey(0)



