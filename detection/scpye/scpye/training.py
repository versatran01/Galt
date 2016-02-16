from __future__ import (print_function, absolute_import, division)
from sklearn.svm import SVC
from sklearn.linear_model import LogisticRegression
from sklearn.grid_search import GridSearchCV
from sklearn.cross_validation import train_test_split
from sklearn.metrics import classification_report
from scpye.image_transformer import *
from scpye.image_pipeline import ImagePipeline, FeatureUnion
from scpye.data_reader import DataReader


def make_image_pipeline(ccw=-1, bbox=None, k=0.5, v_min=25, cspace=None,
                        use_loc=True):
    """
    Factory function for making an image pipeline
    :param ccw: rotate image by ccw
    :param bbox: crop image by bbox
    :param k: resize image by k
    :param v_min: remove dark pixels < v_min
    :param cspace: features - colorspace
    :param use_loc: features - pixel location
    :return: image pipeline
    :rtype: ImagePipeline
    """
    features = make_image_features(cspace, use_loc)

    img_ppl = ImagePipeline([
        ('rotate_image', ImageRotator(ccw)),
        ('crop_image', ImageCropper(bbox)),
        ('resize_image', ImageResizer(k)),
        ('remove_dark', DarkRemover(v_min)),
        ('features', features),
        ('scale', StandardScaler()),
    ])
    return img_ppl


def make_image_features(cspace=None, use_loc=True):
    """
    Factory function for making a feature unin
    :param cspace: features - colorspace
    :param use_loc: features - pixel location
    :return: feature union
    :rtype: FeatureUnion
    """
    if cspace is None:
        cspace = ['hsv']
    transformer_list = [(cs, CspaceTransformer(cs)) for cs in cspace]
    if use_loc:
        transformer_list.append(('mask_location', MaskLocator()))

    # Unfortunately, cannot do a parallel feature extraction
    return FeatureUnion(transformer_list)


def tune_image_classifier(X, y, method='svm', test_size=0.3, report=True):
    """

    :param X:
    :param y:
    :param method:
    :param test_size:
    :param report:
    :return:
    """
    param_grid = [{'C': [0.1, 1, 10]}]
    if method == 'svm':
        clf = SVC()
    elif method == 'lr':
        clf = LogisticRegression()
    else:
        raise ValueError('Unsupported method')

    X_t, X_v, y_t, y_v = train_test_split(X, y, test_size=test_size)
    grid = GridSearchCV(estimator=clf, param_grid=param_grid, cv=4,
                        verbose=5)
    grid.fit(X_t, y_t)

    if report:
        print_grid_search_report(grid)
        print_validation_report(grid, X_v, y_v)

    return grid


def train_image_classifier(data_reader, image_indices, image_pipeline,
                           method='svm'):
    """
    :type data_reader: DataReader
    :param image_indices: list of indices
    :type image_pipeline: ImagePipeline
    :param method:
    :rtype: GridSearchCV
    """
    Is, Ls = data_reader.load_image_label_list(image_indices)
    X_train, y_train = image_pipeline.fit_transform(Is, Ls)
    clf = tune_image_classifier(X_train, y_train, method=method)

    return clf


def print_grid_search_report(grid):
    """
    Print grid search report
    :type grid: GridSearchCV
    """
    print("")
    print("Grid search cross validation report:")
    print("All parameters searched:")
    for params, mean_score, scores in grid.grid_scores_:
        print("{0:03f} (+/-{1:03f}) - {2}".format(mean_score, scores.std() * 2,
                                                  params))
    print("")
    print("Optimal parameters and best score:")
    print("{0:06f} for {1}".format(grid.best_score_, grid.best_params_))
    print("")


def print_validation_report(clf, X, y, target_names=None):
    """
    Print classification report
    :type clf: GridSearchCV
    :type X: numpy.ndarray
    :type y: numpy.ndarray
    :param target_names:
    """
    if target_names is None:
        target_names = ['Non-apple', 'Apple']
    y_p = clf.predict(X)
    report = classification_report(y, y_p, target_names=target_names)
    print(report)
