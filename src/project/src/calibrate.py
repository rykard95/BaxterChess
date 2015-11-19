#! /usr/bin/python
import os, sys, argparse
import numpy as np
import cv2 as v

# Side length of chessboard squares, converted from inches to meters.
SQUARE_DIM = 2.25*2.54/100

def calibrate_camera(imfiles):
    SIZE = (7,7) # number of internal corners, not number of squares

    criteria = (v.TERM_CRITERIA_EPS | v.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Create an array of world coordinates for the chessboard points.
    objp = np.zeros((SIZE[0]*SIZE[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:SIZE[0],0:SIZE[1]].T.reshape(-1,2) * SQUARE_DIM

    # Lists of object and image points from the images.
    objpoints = []
    imgpoints = []

    for imfile in imfiles:
        im = v.imread(imfile, v.CV_LOAD_IMAGE_GRAYSCALE)
        found, corners = v.findChessboardCorners(im, SIZE, None)
        if found:
            objpoints.append(objp)
            imgpoints.append(corners)

            if DBG_SHOW_CORNERS:
                v.drawChessboardCorners(im, SIZE, corners, found)
                v.imshow('Corners', im)
                v.waitKey(500)

    v.destroyAllWindows()

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


