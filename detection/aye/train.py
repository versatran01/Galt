from __future__ import print_function, division, absolute_import
import matplotlib.pyplot as plt
# scikit learn
from sklearn.preprocessing import StandardScaler
from sklearn.cross_validation import train_test_split
from sklearn.externals import joblib
# apple yield estimation
from aye.preprocessing import *
from aye.tune import *

test_size = 0.4

dr = DataReader()
X, y = prepare_data(dr, range(5))

print("Finish loading training data")
print("X: {0}, y: {0}".format(X.shape, y.shape))

# Pre-processing data
print('Scale all data using StandardScaler')
scaler = StandardScaler()
X_scaled = scaler.fit_transform(X)

# Split data into train and test
X_train, X_valid, y_train, y_test = train_test_split(X_scaled, y,
                                                     test_size=test_size)
print('Split data into {0} train and {1} test'.format(len(y_train),
                                                      len(y_test)))

# For now, train an SVC with GridSearchCV
print('Tuning classifiers')
grid = tune_ensemble(X_train[::2], y_train[::2])
print_grid_search_report(grid)

# Validate on y_test
score = grid.score(X_valid, y_test)
print('Test score: {0}'.format(score))

joblib.dump(grid, '../model/ensemble.pkl')
clf = joblib.load('../model/ensemble.pkl')
joblib.dump(scaler, '../model/scaler.pkl')

# Load the final image and see visually how good the classifier is
image, labels_valid = dr.read_image_with_label(5)
s_valid = Samples(image, labels_valid)
X_valid = s_valid.X()

# Apply transform
X_valid_scaled = scaler.transform(X_valid)
y_valid_hat = grid.predict(X_valid_scaled)

plt.figure()
plt.imshow(s_valid.im_bgr)

plt.figure()
bw = s_valid.y_to_bw(y_valid_hat)
plt.imshow(bw, cmap=plt.cm.Greys)
plt.show()
