#!/usr/bin/env python3

import rospy
from hum_det.srv import *
from cv_bridge import CvBridge
import cv2
import numpy as np

NODE_NAME = "stereo_node"

STEREO_MODE_SRV = "stereo_mode"

cv_bridge = CvBridge()


def handle_stereo_mode(req):
	print("===handle_stereo_mode===")
	print("req.bbox.x:", req.bbox.x)
	print("req.bbox.y:", req.bbox.y)
	print("req.bbox.w:", req.bbox.w)
	print("req.bbox.h:", req.bbox.h)
	print("req.disp_img.f:", req.disp_img.f)
	print("req.disp_img.T:", req.disp_img.T)
	print("req.disp_img.image.width:", req.disp_img.image.width)
	print("req.disp_img.image.height:", req.disp_img.image.height)

	disp_img_image = cv_bridge.imgmsg_to_cv2(req.disp_img.image, desired_encoding="32FC1") # to do: использовать для подсчета расстояния
	# потом обязательно удалить {
	# нормализация диспаритета для перевода в 8-битный формат
	norm_disp_img_image = cv2.normalize(disp_img_image, None, 0, 255, cv2.NORM_MINMAX)
	# приведение к uint8
	norm_disp_img_image = np.uint8(norm_disp_img_image)
	# добавление цвета
	norm_disp_img_image = cv2.applyColorMap(norm_disp_img_image, cv2.COLORMAP_JET)
	cv2.imshow(NODE_NAME + "_norm_disp_img_image", norm_disp_img_image)
	cv2.waitKey(1)
	# }

	distance = 5.9
	return StereoModeResponse(distance)


def main() -> None:
	rospy.init_node(NODE_NAME)

	stereo_mode_server = rospy.Service(STEREO_MODE_SRV, StereoMode, handle_stereo_mode)

	rospy.spin()


if __name__ == '__main__':
	main()
