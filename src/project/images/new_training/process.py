from scipy.ndimage import imread
from matplotlib import pyplot as plt
from os import listdir
import numpy as np
from skimage import color, exposure
from scipy.io import savemat
from IPython import embed
from skimage.feature import hog
from scipy import ndimage

classes = ["BlackOnBlack", "BlackOnWhite", "EmptyBlack", "EmptyWhite", "WhiteOnBlack", "WhiteOnWhite"]
# labels = {"BlackOnBlack": np.array([1,0,0,1,0]), "BlackOnWhite": np.array([1,0,0,0,1]),\
#             "EmptyBlack": np.array([0,0,1,1,0]), "EmptyWhite": np.array([0,0,1,0,1]),\
#             "WhiteOnBlack": np.array([0,1,0,1,0]), "WhiteOnWhite": np.array([0,1,0,0,1])}

labels = {"BlackOnBlack": np.array([2]), "BlackOnWhite": np.array([2]),\
            "EmptyBlack": np.array([0]), "EmptyWhite": np.array([0]),\
            "WhiteOnBlack": np.array([1]), "WhiteOnWhite": np.array([1])}



X = []
Y = []

hs = []
f = []
imgs = []
for c in classes:
    #i = 0
    for im in listdir(c):
        image = color.rgb2gray(imread(c+"/"+im))[4:28, 4:28]
        s = np.std(image)
        # image -= np.min(image)
        # image /= np.max(image)
        #if i == 5:     
        fd, h = hog(image, orientations=8, pixels_per_cell=(8,8),\
            cells_per_block=(1,1), visualise=True)
        #fd = image.flatten()
        fd = np.append(fd, s)
        # hs.append(h)
        f.append(fd)
        imgs.append(image)
        #i += 1
        X.append(fd)
        Y.append(labels[c])


X = np.vstack(X)
Y = np.vstack(Y)

shuffle = np.random.randint(0, X.shape[0], X.shape[0])

imgs = [im.flatten() for im in imgs]
imgs = np.vstack(imgs)

X = X[shuffle]
Y = Y[shuffle]
imgs[shuffle]


n = X.shape[0]
a = 9 * n // 10
X_train = X[:a]
Y_train = Y[:a]
X_valid = X[a:]
Y_valid = Y[a:]
imgs= imgs[a:]


data = {'train_images': X_train, 'train_labels': Y_train, 'test_images': X_valid, 'test_labels': Y_valid, 'imgs':imgs}
savemat('data', data)

# # for h in hs:
# #     plt.clf()
# #     plt.imshow(h)
# #     plt.show()

