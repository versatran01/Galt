from __future__ import (print_function, division, absolute_import)

import cv2
import numpy as np
import matplotlib.pyplot as plt


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


def imshow(image, figsize=(10, 10)):
    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111)
    ax.imshow(image, interpolation='none')
    return ax


def imshow2(image1, image2, figsize=(10, 10)):
    fig = plt.figure(figsize=figsize)
    ax1 = fig.add_subplot(121).imshow(image1, interpolation='none')
    ax2 = fig.add_subplot(122).imshow(image2, interpolation='none')
    return ax1, ax2


def imshow3(image1, image2, image3, figsize=(10, 10)):
    fig = plt.figure(figsize=figsize)
    ax1 = fig.add_subplot(131).imshow(image1, interpolation='none')
    ax2 = fig.add_subplot(132).imshow(image2, interpolation='none')
    ax3 = fig.add_subplot(133).imshow(image3, interpolation='none')
    return ax1, ax2, ax3


def draw_bboxes(image, bboxes, color=(255, 0, 0), thickness=1):
    bboxes = np.atleast_2d(bboxes)
    for bbox in bboxes:
        x, y, w, h = np.array(bbox, dtype=int)
        cv2.rectangle(image, (x, y), (x + w, y + h), color=color,
                      thickness=thickness)


def draw_circles(image, circles, color=(255, 0, 0), thickness=1):
    circles = np.atleast_2d(circles)
    for circle in circles:
        x, y, r = np.array(circle, dtype=int)
        cv2.circle(image, (x, y), r, color=color, thickness=thickness)


def draw_points(image, points, color=(255, 0, 0), radius=1):
    points = np.atleast_2d(points)
    for point in points:
        x, y = np.array(point, dtype=int)
        cv2.circle(image, (x, y), radius, color=color, thickness=-1)


def draw_ellipses(image, ellipses, color=(255, 0, 0), thickness=1):
    ellipses = np.atleast_2d(ellipses)
    for ellipse in ellipses:
        x, y, ax1, ax2, ang = np.array(ellipse, dtype=int)
        cv2.ellipse(image, (x, y), (ax1, ax2), ang, 0, 360, color=color,
                    thickness=thickness)


def draw_contours(image, cs, color=(255, 0, 0), thickness=1):
    cv2.drawContours(image, cs, -1, color, thickness)


def draw_contour(image, cnt, color=(255, 0, 0), thickness=1):
    cv2.drawContours(image, [cnt], 0, color, thickness)


def draw_text(image, text, point, color=(255, 0, 0), scale=0.5, thickness=1):
    if type(text) is not str:
        text = str(int(text))

    x, y = np.array(point, dtype=int)
    cv2.putText(image, text, (x, y), 0, scale, color=color,
                thickness=thickness)


def draw_optical_flow(image, points1, points2, color=(255, 0, 0)):
    for pt1, pt2 in zip(points1, points2):
        a, b = pt1.ravel()
        c, d = pt2.ravel()

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
