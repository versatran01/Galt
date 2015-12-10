from __future__ import print_function, division, absolute_import
import matplotlib.pyplot as plt
# scikit learn
from sklearn.preprocessing import StandardScaler
from sklearn.cross_validation import train_test_split
# apple yield estimation
from aye.preprocessing import *
from aye.tune import tune_svc, print_grid_search_report

cwd = os.getcwd()
data_dir = os.path.join(cwd, '..', 'data')

# Get all file names
file_ids = range(1,6)
test_size = 0.3

reader = DataReader()
X = None
y = None
# TODO: make this entire thing a function
for fid in file_ids:
    image, labels = reader.read_image_with_label(fid)
    s = Samples(image, labels)
    X_both, y_both = s.Xy_both()

    if X is None or y is None:
        X = X_both
        y = y_both
    else:
        X = np.vstack((X, X_both))
        y = np.hstack((y, y_both))

print('X: {0}, y: {0}'.format(X.shape, y.shape))

# Pre-processing data
print('Scale all data')
scaler = StandardScaler()
X_scaled = scaler.fit_transform(X)

# Split data into train and test
print('Split data')
X_train, X_valid, y_train, y_test = train_test_split(X_scaled, y,
                                                     test_size=test_size)

# For now, train an SVC with GridSearchCV
print('Tune SVC')
svc_grid = tune_svc(X_train[::2], y_train[::2])

# Optionally, print a verbose report
print_grid_search_report(svc_grid)

# Validate on y_test
score = svc_grid.score(X_valid, y_test)
print('Test score: {0}'.format(score))

# Load the final image and see visually how good the classifier is
image, labels_valid = reader.read_image_with_label(0)
s_valid = Samples(image, labels_valid)
X_valid = s_valid.X()

# Apply transform
X_valid_scaled = scaler.transform(X_valid)
print('X_valid: ', X_valid_scaled.shape)
y_valid_hat = svc_grid.predict(X_valid_scaled)

plt.figure()
plt.imshow(s_valid.im_bgr)

plt.figure()
bw = s_valid.y_to_bw(y_valid_hat)
plt.imshow(bw, cmap=plt.cm.Greys)
plt.show()
