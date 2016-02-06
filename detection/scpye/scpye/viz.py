from __future__ import (print_function, division, absolute_import)

import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


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


def draw_multiple(func):
    def func_wrapper(image, objects, **kwargs):
        objects = np.atleast_2d(objects)
        for obj in objects:
            func(image, obj, **kwargs)

    return func_wrapper


def gray_from_bw(bw, color=False):
    """
    Convert binary image (bool, int) to grayscale image (gray, bgr)
    :param bw: binary image
    :param color: whether to convert to bgr
    :return: grayscale image
    """
    gray = np.array(bw, dtype='uint8') * 255
    if color:
        bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        return bgr
    else:
        return gray


def imshow(image, figsize=(10, 10)):
    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111)
    ax.imshow(image)
    return ax


def imshow2(image1, image2, figsize=(10, 10)):
    fig = plt.figure(figsize=figsize)
    ax1 = fig.add_subplot(121).imshow(image1)
    ax2 = fig.add_subplot(122).imshow(image2)
    return ax1, ax2


@draw_multiple
def draw_bbox(image, bbox, color=(255, 0, 0), thickness=1):
    x, y, w, h = np.array(bbox, dtype=int)
    cv2.rectangle(image, (x, y), (x + w, y + h), color=color,
                  thickness=thickness)


@draw_multiple
def draw_circle(image, circle, color=(255, 0, 0), thickness=1):
    x, y, r = np.array(circle, dtype=int)
    cv2.circle(image, (x, y), r, color=color, thickness=thickness)


@draw_multiple
def draw_point(image, point, color=(255, 0, 0), radius=1):
    x, y = np.array(point, dtype=int)
    cv2.circle(image, (x, y), radius, color=color, thickness=-1)


@draw_multiple
def draw_ellipse(image, ellipse, color=(255, 0, 0), thickness=1):
    x, y, ax1, ax2, ang = np.array(ellipse, dtype=int)
    cv2.ellipse(image, (x, y), (ax1, ax2), ang, 0, 360, color=color,
                thickness=thickness)


def draw_contour(image, cnt, color=(255, 0, 0), thickness=1):
    cv2.drawContours(image, [cnt], 0, color, thickness)


def draw_contours(image, cs, color=(255, 0, 0), thickness=1):
    cv2.drawContours(image, cs, -1, color, thickness)


def draw_text(image, text, point, color=(255, 0, 0), scale=0.5, thickness=1):
    if type(text) is not str:
        text = str(int(text))

    x, y = np.array(point, dtype=int)
    cv2.putText(image, text, (x, y), 0, scale, color=color,
                thickness=thickness)


# Functions start with plot calls pyplot subroutines and usually requires ax
# def plot_filled_bboxes(ax, bboxes, color, alpha):
#     for bbox in bboxes:
#         x, y, w, h = bbox
#         rect = patches.Rectangle((x, y), w, h, facecolor=color, alpha=alpha)
#         ax.add_patch(rect)


# def plot_edge_bboxes(ax, bboxes, color):
#     for bbox in bboxes:
#         x, y, w, h = bbox
#         rect = patches.Rectangle((x, y), w, h, edgecolor=color, fill=False)
#         ax.add_patch(rect)


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
