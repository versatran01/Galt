from __future__ import (print_function, absolute_import, division)
from sklearn.svm import SVC
from sklearn.externals import joblib
from sklearn.grid_search import GridSearchCV
from sklearn.metrics import classification_report
from scpye.image_transformer import (ImageRotator, ImageCropper, ImageResizer,
                                     DarkRemover, CspaceTransformer,
                                     MaskLocator, StandardScaler)
from scpye.image_pipeline import ImagePipeline, FeatureUnion


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
    features = make_feature_union(cspace, use_loc)

    img_ppl = ImagePipeline([
        ('rotate_image', ImageRotator(ccw)),
        ('crop_image', ImageCropper(bbox)),
        ('resize_image', ImageResizer(k)),
        ('remove_dark', DarkRemover(v_min)),
        ('features', features),
        ('scale', StandardScaler()),
    ])
    return img_ppl


def make_feature_union(cspace=None, use_loc=True):
    """
    Factory function for making a feature unin
    :param cspace: features - colorspace
    :param use_loc: features - pixel location
    :return: feature union
    :rtype: FeatureUnion
    """
    if cspace is None:
        cspace = ['bgr', 'hsv']
    transformer_list = [(cs, CspaceTransformer(cs)) for cs in cspace]
    if use_loc:
        transformer_list.append(('mask_location', MaskLocator()))

    # Unfortunately, cannot do a parallel feature extraction
    return FeatureUnion(transformer_list)


def tune_svc(X, y, param_grid, cv=4, verbose=5):
    """
    Tune a support vector machine with cross validation
    :type X: numpy.ndarray
    :type y: numpy.ndarray
    :param param_grid:
    :type param_grid: list
    :param cv: n folds cross validation
    :type cv: int
    :param verbose: verbosity level
    :type verbose: int
    :return: grid search
    :rtype: GridSearchCV
    """
    grid = GridSearchCV(estimator=SVC(), param_grid=param_grid, cv=cv,
                        verbose=verbose)
    grid.fit(X, y)
    return grid


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
