import numpy as np
from scipy.misc import imread
from skimage import color
import pickle
from skimage.feature import hog
def split(image):
    squares = []
    delta = 256/8
    for i in range(8):
        x = i*delta
        for j in range(8):
            y = j*delta
            squares.append(image[x:x+delta, y:y+delta])
    return squares

def featurize(image):
    image = image[4:28, 4:28]
    #return np.append(image.flatten(), np.std(image))
    ret = np.append(hog(image, orientations=8, pixels_per_cell=(8,8),\
           cells_per_block=(1,1), visualise=False), np.std(image)).reshape(1,-1)
    return ret

    
brain = pickle.load(open('pickled_brain.p', 'rb'))
im = color.rgb2gray(imread('frame0000.jpg'))
squares = split(im)
pred = []
for square in squares:
    pred.append(brain.predict(featurize(square)))
pred = np.array(pred)



