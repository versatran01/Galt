import cv2
import numpy as np
import matplotlib.pyplot as plt

filename = '/home/chao/Dropbox/flash.jpg'

image_color = cv2.imread(filename, cv2.IMREAD_COLOR)
image_gray = cv2.cvtColor(image_color, cv2.COLOR_BGR2GRAY)
print(image_gray.shape)

i, j = np.unravel_index(np.argmax(image_gray), image_gray.shape)
cv2.circle(image_color, (j, i), 10, (255, 255, 0), -1)
image_color = cv2.resize(image_color, None, fx=0.5, fy=0.5,
                         interpolation=cv2.INTER_CUBIC)
cv2.imshow('color', image_color)
cv2.waitKey(-1)

# plot the entire row
plt.plot(image_gray[i, :])
plt.show()
