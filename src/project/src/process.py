#! /usr/bin/python
import sys, os
from scipy.ndimage import imread
from matplotlib import pyplot as plt
from os import listdir
import numpy as np
from skimage import color, exposure
from scipy.io import savemat
from skimage.feature import hog
from scipy import ndimage


mode = 'color'
classes = ["BlackOnBlack", "BlackOnWhite", "EmptyBlack", "EmptyWhite", "WhiteOnBlack", "WhiteOnWhite"]

if mode == 'color':
    labels = {"BlackOnBlack": np.array([2]), "BlackOnWhite": np.array([2]),\
            "EmptyBlack": np.array([0]), "EmptyWhite": np.array([0]),\
            "WhiteOnBlack": np.array([1]), "WhiteOnWhite": np.array([1])}
elif mode == 'difference':
    labels = {"BlackOnBlack": np.array([1]), "BlackOnWhite": np.array([2]),\
            "EmptyBlack": np.array([0]), "EmptyWhite": np.array([0]),\
            "WhiteOnBlack": np.array([2]), "WhiteOnWhite": np.array([1])}
X = []
Y = []
images = []
for c in classes:
    print("Currently on class: " + c)
    for im in listdir(os.path.join(sys.argv[1], c)):
        im = os.path.join(sys.argv[1], c, im)
        image = color.rgb2gray(imread(im))[4:28, 4:28]
        images.append(image.flatten())
        s = np.std(image)
        # image -= np.min(image)
        # image /= np.max(image)
        fd = hog(image, orientations=8, pixels_per_cell=(8,8), \
                 cells_per_block=(1,1))
        fd = np.append(fd, s)
        X.append(fd)
        Y.append(labels[c])

X = np.vstack(X)
Y = np.vstack(Y)
images = np.vstack(images)
shuffle = np.random.randint(0, X.shape[0], X.shape[0])

print("Finished Stacking!")
X = X[shuffle]
Y = Y[shuffle]


n = X.shape[0]
a = 9 * n // 10
X_train = X[:a]
Y_train = Y[:a]
X_valid = X[a:]
Y_valid = Y[a:]
images = images[shuffle][:a]


data = {'train_images': X_train, 'train_labels': Y_train, 'test_images': X_valid, 'test_labels': Y_valid, 'mode':mode, 'images': images}
savemat('data.mat', data)

