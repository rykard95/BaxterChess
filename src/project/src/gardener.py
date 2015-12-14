#! /usr/bin/python
from scipy.io import loadmat
import matplotlib.pyplot as plt
from numpy.random import randint
from sklearn.ensemble import AdaBoostClassifier,\
        RandomForestClassifier
from sklearn.tree import DecisionTreeClassifier
import numpy as np
import matplotlib.cm as cm
import pickle
from sklearn.svm import SVC


data = loadmat('data.mat')

MODE = data['mode'] 
print("Preparing soil.")
train_images = data['train_images']
n = train_images.shape[0]
train_labels = data['train_labels'].flatten()

test_images = data['test_images']
test_labels = data['test_labels'].flatten()


# 250
print("Planting trees!\n")
# fc = AdaBoostClassifier(DecisionTreeClassifier(max_depth=4),
#                         n_estimators=200, learning_rate=0.5)
fc = RandomForestClassifier(n_estimators=500, bootstrap=True, n_jobs=4)

print("Growing trees!\n")
#bdt.fit(train_images, train_labels)
fc.fit(train_images, train_labels)

print("The trees are thinking!")
#pred = bdt.predict(test_images)
pred = fc.predict(test_images)
#pred_prob = bdt.predict_proba(test_images)
pred_prob = fc.predict_proba(test_images)

# clf = SVC(C=200)
# clf.fit(train_images, train_labels)
# pred = clf.predict(test_images)
acc = (pred == test_labels).astype(int)
print("The accuracy is " + str(float(sum(acc))/len(acc)))

ind = np.where(acc == 0)
print("wrong guesses: " + str(pred[ind]))
print("actual labels: " + str(test_labels[ind]))
#print("probabilities: " + str(pred_prob[ind]))

for i in ind[0]:
    lreal = np.log(pred_prob[i][test_labels[i]])
    lpred = np.log(pred_prob[i][pred[i]])
    print pred[i], test_labels[i], lreal - lpred

#richard = data['images'][ind]
# for im in richard:
    #plt.imshow(im.reshape((24,24)), cmap=plt.cm.gray)
    #plt.show()


print("Recalling training set")
pred = fc.predict(train_images)
acc = (pred == train_labels).astype(int)
print("The accuracy is " + str(float(sum(acc))/len(acc)))

fc.mode = MODE
pickle.dump(fc, open('pickled_brain.p', 'wb'))
