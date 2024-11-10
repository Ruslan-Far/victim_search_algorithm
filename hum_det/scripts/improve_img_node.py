#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

NODE_NAME = "improve_img_node"
IMG_SUB_TOPIC = "/usb_cam_node/image_raw"
FREQ = 10 # ВАЖНО!!! Иначе (если прослушивать топики камеры инженера удаленно по сети) будет очень сильно зависать и будет очень мало разнообразных изображений!
img_callback_count = 0


def img_callback(msg: Image, cv_bridge: CvBridge) -> None:
	global img_callback_count

	# img_bgr = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
	img_bgr = cv_bridge.imgmsg_to_cv2(msg)
	img_rgb = img_bgr

	if img_callback_count == 0:
		denoised_image = cv2.fastNlMeansDenoisingColored(img_rgb, None, 3, 3, 5, 15)
		# Повышение резкости с использованием фильтра
		kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
		sharpened_image = cv2.filter2D(denoised_image, -1, kernel)
		# Увеличение контраста
		lab = cv2.cvtColor(sharpened_image, cv2.COLOR_BGR2LAB)
		l, a, b = cv2.split(lab)
		clahe = cv2.createCLAHE(clipLimit=1.5, tileGridSize=(8, 8))
		cl = clahe.apply(l)
		limg = cv2.merge((cl, a, b))
		final_image = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)

		publisher = rospy.Publisher("/improved" + IMG_SUB_TOPIC, Image, queue_size=1)
		final_image = cv_bridge.cv2_to_imgmsg(final_image, "rgb8")
		publisher.publish(final_image)
	img_callback_count += 1
	if img_callback_count == FREQ:
		img_callback_count = 0


def main() -> None:
	rospy.init_node(NODE_NAME)

	cv_bridge: CvBridge = CvBridge()

	rospy.Subscriber(IMG_SUB_TOPIC, Image, lambda msg: img_callback(msg, cv_bridge), queue_size = None)

	rospy.spin()


if __name__ == '__main__':
	main()