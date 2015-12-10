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
file_ids = range(5)
file_ext = 'png'

reader = DataReader()

k = 0.25
v_thresh = 20
test_size = 0.3
X = None
y = None
for fid in file_ids:
    images, labels = reader.read_cspace_with_label(fid, k)
    X_both, y_both = make_train_samples_features(images, labels, 20)

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
svc_grid = tune_svc(X_train[::4], y_train[::4])

# Optionally, print a verbose report
print_grid_search_report(svc_grid)

# Validate on y_test
score = svc_grid.score(X_valid, y_test)
print('Test score: {0}'.format(score))

# Load the final image and see visually how good the classifier is
images_valid, labels_valid = reader.read_cspace_with_label(5, k)
# X_valid = make_test_samples(images_valid)
X_valid, mask = make_masked_test_samples(images_valid, 20)

# Apply transform
X_valid_scaled = scaler.transform(X_valid)
print('X_valid: ', X_valid_scaled.shape)
y_valid_hat = svc_grid.predict(X_valid_scaled)

plt.figure()
im_bgr_valid = images_valid[0]
plt.imshow(im_bgr_valid)

# plt.figure()
# print(y_valid_hat.shape)
# bw_test = np.reshape(y_valid_hat, labels_valid[0].shape)
# bw_test = bw_test > 0
# plt.imshow(bw_test, cmap=plt.cm.Greys)
# plt.show()

plt.figure()
bw = masked_y_to_image(y_valid_hat, mask)
plt.imshow(bw, cmap=plt.cm.Greys)
plt.show()

# Maybe calculate how good it is doing
