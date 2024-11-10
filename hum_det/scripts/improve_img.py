#!/usr/bin/env python3

import cv2
import numpy as np

# Загрузка изображения
# image = cv2.imread("/home/ruslan/kpfu/magistracy/test_images/room1412_1_frame0028.jpg")
# image = cv2.imread("/home/ruslan/kpfu/magistracy/test_images/room1412_5_frame0052.jpg")
# image = cv2.imread("/home/ruslan/kpfu/magistracy/test_images/room1412_7_frame0129.jpg")
image = cv2.imread("/home/ruslan/kpfu/magistracy/test_images/room1412_3_frame0010.jpg")
# image = cv2.imread("/home/ruslan/kpfu/magistracy/test_images/screen_room1412_1_Screenshot from 2024-10-12 17-13-15.png")

# Уменьшение шума с использованием фильтра размытия
# denoised_image = cv2.fastNlMeansDenoisingColored(image, None, 10, 10, 7, 21)
# denoised_image = cv2.fastNlMeansDenoisingColored(image, None, 3, 3, 7, 21)
# denoised_image = cv2.fastNlMeansDenoisingColored(image, None, 3, 3, 5, 21)
denoised_image = cv2.fastNlMeansDenoisingColored(image, None, 3, 3, 5, 15)

# Повышение резкости с использованием фильтра
kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
# kernel = np.array([[0, -1.1, 0], [-1.1, 5, -1.1], [0, -1.1, 0]])
sharpened_image = cv2.filter2D(denoised_image, -1, kernel)

# Увеличение контраста
lab = cv2.cvtColor(sharpened_image, cv2.COLOR_BGR2LAB)
l, a, b = cv2.split(lab)
# clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
clahe = cv2.createCLAHE(clipLimit=1.5, tileGridSize=(8, 8))
# clahe = cv2.createCLAHE(clipLimit=1.5, tileGridSize=(1, 1))
# clahe = cv2.createCLAHE(clipLimit=10.0, tileGridSize=(8, 8))
cl = clahe.apply(l)
limg = cv2.merge((cl, a, b))
final_image = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)

cv2.imshow("improve_img", final_image)
cv2.waitKey(0)

# Сохранение результата
# cv2.imwrite("/home/ruslan/kpfu/magistracy/test_images/enhanced_room1412_1_frame0028.jpg", final_image)
# cv2.imwrite("/home/ruslan/kpfu/magistracy/test_images/enhanced_room1412_5_frame0052.jpg", final_image)
# cv2.imwrite("/home/ruslan/kpfu/magistracy/test_images/enhanced_room1412_7_frame0129.jpg", final_image)
# cv2.imwrite("/home/ruslan/kpfu/magistracy/test_images/enhanced_screen_room1412_1_Screenshot from 2024-10-12 17-13-15.png", final_image)
