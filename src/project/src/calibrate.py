#! /usr/bin/python
import os, sys, argparse
import numpy as np
import cv2 as v

# Side length of chessboard squares, converted from inches to meters.
SQUARE_DIM = 2.25*2.54/100
# number of internal corners, not number of squares
BOARD_SIZE = (7,7) 

# Various parameters for edge refinement algorithm.
CRNR_TERM = (v.TERM_CRITERIA_EPS | v.TERM_CRITERIA_MAX_ITER, 30, 0.001)
CRNR_WIND = (5,5)
CRNR_IGN = (-1,-1)


def calibrate_camera(imfiles):
    """Given a list of calibration image filenames IMFILES, compute
    and return the camera matrix and distortion parameters."""

    # Create an array of world coordinates for the chessboard points.
    size = BOARD_SIZE
    objp = np.zeros((size[0]*size[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:size[0],0:size[1]].T.reshape(-1,2) * SQUARE_DIM

    # Lists of object and image points from the images.
    objpoints = []
    imgpoints = []

    # For each image in the calibration set, find the chessboard and
    #  add the detected points to objpoints and imgpoints.
    for imfile in imfiles:

        im = v.imread(imfile, v.CV_LOAD_IMAGE_GRAYSCALE)
        found, corners = v.findChessboardCorners(im, SIZE, None)

        if found:
            # refine the corners
            v.cornerSubPix(im, corners, CRNR_WIND, CRNR_IGN, CRNR_CRIT)

            # then add them to the corner list
            objpoints.append(objp)
            imgpoints.append(corners)

            # If verbose, show the corners as they're detected.
            if DBG_SHOW_CORNERS:
                v.drawChessboardCorners(im, SIZE, corners, found)
                v.imshow('Corners', im)
                v.waitKey(500)

    v.destroyAllWindows()

    # Find camera calibration coefficients based on all those points.
    _, mtx, dist, _, _ = v.calibrateCamera(objpoints, imgpoints, 
                                           im.shape[::-1], None, None)

    return mtx, dist


def get_arguments():
    desc = 'Vastly improved camera calibration script. :)'
    parser = argparse.ArgumentParser(description = desc)
    parser.add_argument('filenames', metavar='IMAGE', type=str, nargs='+',
                        help='calibration image filenames')
    parser.add_argument('-o', '--output', required=True,
                        help='output image name', metavar='OUT')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='show verbose debug images')
    return parser.parse_args()

if __name__ == '__main__':
    args = get_arguments()
    imfiles = args.filenames
    outfile = args.output
    DBG_SHOW_CORNERS = args.verbose

    mtx, dist = calibrate_camera(imfiles)
    np.savez(outfile, mtx=mtx, dist=dist)


