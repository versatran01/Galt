from __future__ import (absolute_import, division, print_function)
import cv2
import numpy as np
from scpye.visualization import imshow2
from scpye.data_reader import DataReader
from scpye.image_pipeline import ImagePipeline
from scpye.region_props import find_contours, gray_from_bw
from scpye.visualization import draw_contours


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

        bw_pos = get_positive_bw(image_pipeline, I, L)
        bw_pos = gray_from_bw(bw_pos)
        cntrs_pos = find_contours(bw_pos)

        bw_pred = get_prediction_bw(image_pipeline, image_classifier, I)
        bw_pred = gray_from_bw(bw_pred)

        # Draw contour of labeled apple
        disp_label = cv2.cvtColor(bw_pred, cv2.COLOR_GRAY2BGR)
        draw_contours(disp_label, cntrs_pos)

        disp_color = image_pipeline.named_steps['remove_dark'].bgr
        imshow2(disp_color, disp_label, figsize=(17, 17))
