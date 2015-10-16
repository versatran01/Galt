import cv2
import rospkg
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

filename = '../image/frame0000.jpg'

image_color = cv2.imread(filename, cv2.IMREAD_COLOR)
image_gray = cv2.cvtColor(image_color, cv2.COLOR_BGR2GRAY)
print(image_gray.shape)

image_gray = cv2.resize(image_gray, None, fx=0.2, fy=0.2,
                        interpolation=cv2.INTER_NEAREST)
image_gray = cv2.GaussianBlur(image_gray, (7, 7), 0)

height, width = image_gray.shape
X = np.arange(0, width, 1)
Y = np.arange(0, height, 1)
X, Y = np.meshgrid(X, Y)
Z = image_gray

fig = plt.gcf()
ax = fig.gca(projection='3d')
surf = ax.plot_surface(X, Y, Z, cstride=int(width / 20),
                       rstride=int(height / 20),
                       cmap=cm.jet)
ax.set_aspect('equal')
# ax = fig.gca()
# ax.contour(X, Y, Z)
# i, j = np.unravel_index(np.argmax(image_gray), image_gray.shape)
# cv2.circle(image_color, (j, i), 10, (255, 255, 0), -1)
# image_color = cv2.resize(image_color, None, fx=0.5, fy=0.5,
#                          interpolation=cv2.INTER_CUBIC)
# cv2.imshow('color', image_color)
# cv2.waitKey(-1)

# plot the entire row
plt.show()
