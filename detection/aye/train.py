from __future__ import print_function, division, absolute_import
# common
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
# scikit learn
from sklearn.preprocessing import StandardScaler
# apple yield estimation
from aye.preprocessing import resize_image, convert_image_colorspace
from aye.tune import tune_svc, print_grid_search_report

cwd = os.getcwd()
data_dir = os.path.join(cwd, '..', 'data')

# Get all file names
file_ids = range(5)
file_ext = 'png'

k = 0.25
v_thresh = 20
X = []
Y = []
for fid in file_ids:
    # Get file name
    # f_raw, f_pos, f_neg = get_file_name(fid, file_ext)

    filename_raw = "frame{0:04d}.{1}".format(fid, file_ext)
    filename_pos = "frame{0:04d}_positive.{1}".format(fid, file_ext)
    filename_neg = "frame{0:04d}_negative.{1}".format(fid, file_ext)

    # Load all images
    im_bgr = cv2.imread(filename_raw, cv2.IMREAD_COLOR)
    im_pos = cv2.imread(filename_pos, cv2.IMREAD_GRAYSCALE)
    im_neg = cv2.imread(filename_neg, cv2.IMREAD_GRAYSCALE)

    # Resize images
    k = 0.4
    im_pos = resize_image(im_pos, k)
    im_neg = resize_image(im_neg, k)
    im_bgr = resize_image(im_bgr, k)

    # Convert image to different colorspaces
    im_hsv = convert_image_colorspace(im_bgr, 'hsv')
    im_lab = convert_image_colorspace(im_bgr, 'lab')

    # Filter data based on v value in hsv
    v = im_hsv[:, :, -1]
    mask_v = (v > v_thresh) & (v < 255 - v_thresh)
    mask_pos = im_pos > 0
    mask_neg = im_neg > 0

    mask_pos_v = mask_pos & mask_v
    mask_neg_v = mask_neg & mask_v

    # Extract data from images
    X_pos_bgr = im_bgr[mask_pos_v]
    X_pos_hsv = None
    X_pos_lab = None

    X_neg_bgr = None
    X_neg_hsv = None
    X_neg_lab = None

    X_pos = np.hstack((X_pos_bgr, X_pos_hsv, X_pos_lab))
    X_neg = np.hstack((X_neg_bgr, X_neg_hsv, X_neg_lab))

    # Generate corresponding labels
    Y_pos = None
    Y_neg = None

    # Append X_pos, Y_pos to X, Y
    # TODO:

# Pre-processing data
scaler = StandardScaler()
X = scaler.fit_transform(X)

# Split data into train and test
X_train = []
X_test = []
Y_train = []
Y_test = []

# For now, train an SVC with GridSearchCV
svc_grid = tune_svc(X, Y)

# Optionally, print a verbose report
print_grid_search_report(svc_grid)

# Load the final image and see visually how good the classifier is

# Maybe calculate how good it is doing
