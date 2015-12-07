#! /usr/bin/python
import os
import numpy as np
import cv2 as v
import argparse
from IPython import embed
from scipy.misc import imsave, imread
from skimage import color

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

    num_to_folder = {1: "BlackOnBlack/",\
                        2:"BlackOnWhite/",\
                        3:"EmptyBlack/",\
                        4:"EmptyWhite/",\
                        5:"WhiteOnBlack/",\
                        6:"WhiteOnWhite/"}
    # load the image and undistort it
    for board in filenames:
        print(board)

        img = imread(board)

        squares = []
        # the width of each square in unwarped
        delta = 32
        # the number of border pixels on each side to drop out; I used 8
        #  earlier to ignore the border, but I think 0 makes sense for a 
        #  classifier with a little more brains than just np.std
        for i in range(8):
            y = i*delta
            for j in range(8):
                x = j*delta
                a = img[x:x+delta, y:y+delta]
                imsave(str(num) + ".jpg", a)
                num += 1
        # squares.sort(key=evaluate_square)
        # for square in squares:
        #     imsave(str(num) + ".jpg", square)
        #     num += 1

        # Create and display a single 8x8 board image of just the squares 


