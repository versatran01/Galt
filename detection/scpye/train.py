from __future__ import print_function, division, absolute_import

import matplotlib.pyplot as plt
# scikit learn
from sklearn.preprocessing import StandardScaler
from sklearn.cross_validation import train_test_split
from sklearn.externals import joblib
# apple yield estimation
from scpye.preprocessing import *
from scpye.tune import *


class Trainer:
    def __init__(self, apple, k=0.5, n_images=5, test_size=0.4, use_ind=True):
        self.apple = apple
        self.k = k
        self.n_images = n_images
        self.test_size = test_size
        self.use_ind = use_ind
        self.data_dir = os.path.join('../data', self.apple)
        self.model_dir = os.path.join('../model', self.apple)
        self.reader = DataReader(rel_dir=self.data_dir)
        if self.apple == 'green':
            self.roi = [0, 200, 1440, 800]
        elif self.apple == 'red':
            self.roi = [240, 200, 1440, 800]
        else:
            raise ValueError("Invalid apple")

        self.clf = None
        self.scaler = None
        self.score = None

    def train(self):
        # TODO: THIS IS FUCKING HACK
        # Load raw data
        X, y = prepare_data(self.reader, range(self.n_images), self.roi, self.k,
                            self.use_ind)

        print("Finish loading training data")
        print("X: {0}, y:{1}".format(X.shape, y.shape))
        n_y = np.count_nonzero(y)
        print("Pos: {0}, Neg: {1}".format(n_y, np.size(y) - n_y))

        # Pre-processing data
        print('Scale all data using StandardScaler')
        self.scaler = StandardScaler()
        X_scaled = self.scaler.fit_transform(X)

        # Split data into train and test
        X_t, X_v, y_t, y_v = train_test_split(X_scaled, y,
                                              test_size=self.test_size)
        print('Train: {0}, Valid: {1}'.format(len(y_t), len(y_v)))

        # For now, train an SVC with GridSearchCV
        print('Tuning classifiers')
        # Just to speed things up
        self.clf = tune_svc(X_t[::2], y_t[::2])
        print_grid_search_report(self.clf)

        # Validate on y_test
        self.score = self.clf.score(X_v, y_v)
        print('Test score: {0}'.format(self.score))

    def save(self):
        assert self.clf is not None or self.scaler is not None
        joblib.dump(self.clf, os.path.join(self.model_dir, 'svc.pkl'))
        joblib.dump(self.scaler, os.path.join(self.model_dir, 'scaler.pkl'))

    def validate(self):
        image, labels_valid = self.reader.read_image_with_label(self.n_images)
        s_v = Samples(image, labels_valid, self.roi, self.k, self.use_ind)
        X_v = s_v.X()
        X_v_scaled = self.scaler.transform(X_v)
        y_v_hat = self.clf.predict(X_v_scaled)

        fig = plt.figure(figsize=(16, 16))
        ax = fig.add_subplot(121)
        ax.imshow(s_v.im_bgr)
        ax = fig.add_subplot(122)
        bw = s_v.y_to_bw(y_v_hat, to_gray=True)
        ax.imshow(bw, cmap=plt.cm.gray)
        return s_v, bw


if __name__ == '__main__':
    red = Trainer('red', k=0.5, n_images=7, test_size=0.4, use_ind=True)
    red.train()
    red.save()

    green = Trainer('green', k=0.5, n_images=7, test_size=0.4, use_ind=True)
    green.train()
    green.save()
