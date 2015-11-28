#! /usr/bin/python
import os
import numpy as np
import cv2 as v
import argparse


def seek_to(file, content):
    """
    Given a file, seeks to the line immediately after the first one 
    containing the string CONTENT. After reaching the end of the file, 
    seeks back to the beginning and keeps looking up to the point we 
    started at. Return the last line read if it worked, or an empty 
    string if not found.
    """
    point = file.tell()
    has_looped = False

    while True:
        line = file.readline()
        if content in line: return line

        if line == '':
            file.seek(0)
            has_looped = True
        elif has_looped and file.tell() >= point:
            return ''
    

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



def find_chessboard_corners(image, scale=1):
    """
    Given an already-loaded image and a percentage to scale by, tries to 
    find a chessboard in the scaled version of the image, then find the
    points to subpixel precision in the original image.
    """
    if scale != 1:
        smaller = v.resize(image, dstsize=None, fx=scale, fy=scale)

    found, corners = v.findChessboardCorners(image, (7,7), None)

    corners = corners / scale

    return found, corners



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


def label_squares(squares):
    """
    Given a list of images of board squares, return a string specifying 
    the label of each in order.
    Currently a dumb implementation that assumes all squares with more
    standard deviation than the median are occupied.
    """
    std = [np.std(square[:,:,2]) for square in squares]
    med = np.median(std)

    labels = []
    for i in range(len(squares)):
        if std[i] > med:
            labels.append(1)
        else:
            labels.append(0)
    return labels


if __name__ == '__main__':

    # read command line
    parser = argparse.ArgumentParser()
    parser.add_argument('calib', help='calibration parameters file to use')
    parser.add_argument('filenames', metavar='IMAGE', nargs='+',
                        help='board image file')
    parser.add_argument('-w', '--write', action='store_true',
                        help='create output image file for each square')
    parser.add_argument('-l', '--labels',
                        help='file of image labels')
    args = parser.parse_args()
    calib = args.calib
    filenames = args.filenames
    labels = open(args.labels, 'r') if args.labels else None
    WRITE = args.write

    # load the image and undistort it
    mtx, dist = load_camera_calibration(calib)
    total_accuracy = 0
    for board in filenames:
        print 'Investigating', board

        img = undistorted_imread(mtx, dist, board)
        bw = img # v.cvtColor(img, v.COLOR_RGB2GRAY)

        # find the chessboard in the undistorted image
        found, corners = v.findChessboardCorners(bw, (7,7), None)

        if found:
            # sharpen the corners
            crit = (v.TERM_CRITERIA_EPS | v.TERM_CRITERIA_MAX_ITER, 30, 0.1)
            # v.cornerSubPix(bw, corners, (11,11), (-1,-1), crit)

            # find the perspective transform
            size = 512
            pts1, pts2 = getChessboardOutsideCorners(corners, size)
            M = v.getPerspectiveTransform(pts1, pts2)
            unwarped = v.warpPerspective(bw, M, (size,size))

        if not found: 
            print 'Argh, no can find!'
            continue

        # a list of all the chessboard squares; should be in order from
        # top left to bottom right just like the labels
        squares = []
        # the width of each square in unwarped
        delta = size/8
        # the number of border pixels on each side to drop out; I used 8
        #  earlier to ignore the border, but I think 0 makes sense for a 
        #  classifier with a little more brains than just np.std
        ign = 8
        for i in range(8):
            x = i*delta
            for j in range(8):
                y = j*delta
                squares.append(unwarped[x+ign:x+delta-ign, 
                                        y+ign:y+delta-ign])

        if labels is None:
            # Create and display a single 8x8 image of just the squares 
            squaresprime = []
            for i in range(0,64,8):
                squaresprime.append(np.hstack(squares[i:i+8]))
            out = np.vstack(squaresprime)

            # Display the grid
            print '\tdone'
            v.imshow('out', out)
            v.waitKey(0)

            continue

        # Grab the labels of each of the squares from the labels file.
        seek_to(labels, board)
        string_real_label = ''
        for i in range(8):
            string_real_label += labels.readline().strip()
        real_label = [1 if c in 'bwBW' else 0 for c in string_real_label]

        # Compute the labels of each of the squares, and test correctness.
        test_label = label_squares(squares)
        correctness = [1.0 * (test_label[i] == real_label[i]) 
                       for i in range(len(squares))]
        accuracy = sum(correctness) / len(correctness)

        print '\tlabels', accuracy*100, 'percent accurate'
        total_accuracy += accuracy

    if labels: 
        print 'Total accuracy:', total_accuracy/len(filenames)
            



