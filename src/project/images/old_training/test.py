import glob
import cv2 as v
import numpy as np
from skimage.transform import (hough_line, hough_line_peaks)

names = glob.glob('*.jpg')
bws = [v.imread(name, v.CV_LOAD_IMAGE_GRAYSCALE) for name in names]
ims = [v.imread(name, v.CV_LOAD_IMAGE_COLOR) for name in names]

def adaptiveThreshold(im):
    return v.adaptiveThreshold(im, 255, v.ADAPTIVE_THRESH_MEAN_C, 
                               v.THRESH_BINARY, BLOCK_SIZE, 0)

def watershed(im, at):
    out = np.zeros(at.shape, dtype=np.uint32)
    out += at
    v.watershed(im, out)
    return out

def seek(module, name):
    return filter(lambda x: name in x, dir(module))

def show_hough_lines(im):
    bw = v.cvtColor(im, v.COLOR_BGR2GRAY)
    at = adaptiveThreshold(bw)
    can = v.Canny(at, 32, 256-32)
    hough = v.HoughLinesP(can, 1, 3.1415/180, 80, 300, 30)

    copied = im[:,:,:]
    for i in range(hough.shape[1]):
        v.line(copied, (hough[0,i,0], hough[0,i,1]), 
                       (hough[0,i,2],hough[0,i,3]), 
                       (0,0,255), 3, 8)
    return copied

def ski_hough_lines(im):
    h, theta, d = hough_line(im)

