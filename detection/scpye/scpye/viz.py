from __future__ import (print_function, division, absolute_import)

import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


# TODO: get rid of all these for loops using higher-order functions

class Colors:
    """
    Collection of colors
    """

    predict = (0, 0, 255)  # blue
    detect = (255, 0, 0)  # red
    correct = (255, 255, 0)  # yellow
    new = (255, 255, 255)  # white
    match = (0, 255, 0)  # green
    flow = (255, 0, 0)  # red
    text = (0, 255, 255)

    def __init__(self):
        pass


def imshow(image, fsize=(10, 10), title=""):
    """
    Convenient function to display image
    :param image:
    :param title:
    :param fsize:
    """
    fig = plt.figure(figsize=fsize)
    ax = fig.add_subplot(111)
    ax.imshow(image)
    return ax


def imshow2(image1, image2, fsize=(10, 10)):
    """
    Convenient function to display 2 images side by side
    :param image1:
    :param image2:
    :param fsize:
    """
    fig = plt.figure(figsize=fsize)
    ax1 = fig.add_subplot(121).imshow(image1)
    ax2 = fig.add_subplot(122).imshow(image2)
    return ax1, ax2


def draw_contours(image, cs, color=(255, 0, 0), thickness=1):
    """
    Draw a list of contours on image
    :param image:
    :param cs:
    :param color:
    :param thickness:
    :return:
    """
    cv2.drawContours(image, cs, -1, color, thickness)


def draw_contour(image, cnt, color=(255, 0, 0), thickness=1):
    """
    Draw a single contour on image
    :param image:
    :param cnt:
    :param color:
    :param thickness:
    """
    cv2.drawContours(image, [cnt], 0, color, thickness)


def draw_bbox(image, bbox, color=(255, 0, 0), thickness=1):
    """
    Draw a single bounding box on image
    :param image:
    :param bbox: [x, y, w, h]
    :param color:
    :param thickness:
    """
    x, y, w, h = np.array(bbox, dtype=int)
    cv2.rectangle(image, (x, y), (x + w, y + h), color=color,
                  thickness=thickness)


def draw_circle(image, circle, color=(255, 0, 0), thickness=1):
    """
    Draw a single circle on image
    :param image:
    :param circle:
    :param color:
    :param thickness:
    """
    x, y, r = np.array(circle, dtype=int)
    cv2.circle(image, (x, y), r, color=color, thickness=thickness)


def draw_ellipse(image, ellipse, color=(255, 0, 0), thickness=1):
    """
    Draw a single ellipse on image
    :param image:
    :param ellipse:
    :param color:
    :param thickness:
    """
    x, y, a1, a2, an = np.array(ellipse, dtype=int)
    cv2.ellipse(image, (x, y), (a1, a2), an, 0, 360, color=color,
                thickness=thickness)


def draw_text(image, text, point, color=(255, 0, 0), scale=0.5, thickness=1):
    """
    Draw text at point on image
    :param image:
    :param text:
    :param point:
    :param color:
    :param scale:
    :param thickness:
    """
    if type(text) is not str:
        text = str(int(text))

    x, y = np.array(point, dtype=int)
    cv2.putText(image, text, (x, y), 0, scale, color=color,
                thickness=thickness)


# Functions start with plot calls pyplot subroutines and usually requires ax
def plot_filled_bboxes(ax, bboxes, color, alpha):
    for bbox in bboxes:
        x, y, w, h = bbox
        rect = patches.Rectangle((x, y), w, h, facecolor=color, alpha=alpha)
        ax.add_patch(rect)


def plot_edge_bboxes(ax, bboxes, color):
    for bbox in bboxes:
        x, y, w, h = bbox
        rect = patches.Rectangle((x, y), w, h, edgecolor=color, fill=False)
        ax.add_patch(rect)


# Functions start with draw calls opencv subroutines
def draw_bboxes(image, bboxes, color, thickness=1):
    bboxes = np.atleast_2d(bboxes)
    bboxes = np.array(bboxes, copy=True, dtype=int)
    for bbox in bboxes:
        x, y, w, h = bbox
        cv2.rectangle(image, (x, y), (x + w, y + h),
                      color=color, thickness=thickness)


def draw_optical_flow(image, p1, p2, color=(255, 0, 0)):
    for p1g, p2g in zip(p1, p2):
        a, b = p1g.ravel()
        c, d = p2g.ravel()

        cv2.line(image, (a, b), (c, d), color=color, thickness=1)
        cv2.circle(image, (c, d), 1, color=color, thickness=-1)


def draw_bboxes_matches(image, matches, bboxes1, bboxes2, color, thickness=1):
    matches = np.atleast_2d(matches)
    for pair in matches:
        i1, i2 = pair
        b1 = bboxes1[i1]
        b2 = bboxes2[i2]
        x1, y1, w1, h1 = b1
        x2, y2, w2, h2 = b2
        a = int(x1 + w1 / 2)
        b = int(y1 + h1 / 2)
        c = int(x2 + w2 / 2)
        d = int(y2 + h2 / 2)
        cv2.line(image, (a, b), (c, d), color=color, thickness=thickness)