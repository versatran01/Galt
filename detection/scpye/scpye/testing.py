from __future__ import (absolute_import, division, print_function)
import cv2
import numpy as np
from scpye.blob_analysis import gray_from_bw
from scpye.visualization import imshow2
from scpye.data_reader import DataReader
from scpye.image_pipeline import ImagePipeline
from scpye.blob_analysis import find_contours
from scpye.visualization import draw_contours


# TODO: All these stuff should be integrated into FruitDetector
def get_positive_bw(image_pipeline, image, label):
    image_pipeline.transform(image, label)
    label = image_pipeline.named_steps['remove_dark'].label
    pos = label[:, :, 1]
    return pos


def get_prediction_bw(image_pipeline, image_classifier, image):
    # Get prediction
    X = image_pipeline.transform(image)
    y = image_classifier.predict(X)
    bw = image_pipeline.named_steps['remove_dark'].mask.copy()
    bw[bw > 0] = y
    return bw


def test_image_classifier(data_reader, image_indices, image_pipeline,
                          image_classifier):
    """
    :type data_reader: DataReader
    :param image_indices:
    :type image_pipeline: ImagePipeline
    :param image_classifier:
    :return:
    """
    if np.isscalar(image_indices):
        image_indices = [image_indices]

    for ind in image_indices:
        I, L = data_reader.load_image_label(ind)

        pos_bw = get_positive_bw(image_pipeline, I, L)
        pos_bw = gray_from_bw(pos_bw)
        pos_cntrs = find_contours(pos_bw)

        pred_bw = get_prediction_bw(image_pipeline, image_classifier, I)
        pred_bw = gray_from_bw(pred_bw)

        # Draw contour of labeled apple
        disp_label = cv2.cvtColor(pred_bw, cv2.COLOR_GRAY2BGR)
        draw_contours(disp_label, pos_cntrs)

        disp_color = image_pipeline.named_steps['remove_dark'].image
        imshow2(disp_color, disp_label)
