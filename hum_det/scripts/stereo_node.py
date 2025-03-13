#!/usr/bin/env python3

import rospy
from hum_det.srv import *
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image

NODE_NAME = "stereo_node"

STEREO_MODE_SRV = "stereo_mode"

norm_depth_map_pub = rospy.Publisher("/stereo_node/norm_depth_map", Image, queue_size=1)

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
	print("req.disp_img.min_disparity:", req.disp_img.min_disparity)
	print("req.disp_img.max_disparity:", req.disp_img.max_disparity)

	disp_img_image = cv_bridge.imgmsg_to_cv2(req.disp_img.image, desired_encoding="32FC1") # to do: использовать для подсчета расстояния
	roi = disp_img_image[req.bbox.y:req.bbox.y+req.bbox.h, req.bbox.x:req.bbox.x+req.bbox.w]
	# req.disp_img.min_disparity должен быть >= 1
	depth_map = np.where(roi >= req.disp_img.min_disparity, (req.disp_img.f * req.disp_img.T) / roi, 0)
	valid_pixels = depth_map[depth_map > 0]
	if valid_pixels.size > 0:
		median_distance = np.median(valid_pixels) # медиана
		mean_distance = np.mean(valid_pixels) # среднее значение
		print(f"медианная дистанция: {median_distance}")
		print(f"средняя дистанция: {mean_distance}")
		distance = mean_distance
	else:
		print("нет валидных пикселей в roi!")
		distance = 0
	print(f"valid_pixels.size: {valid_pixels.size}")
	# потом обязательно удалить {
	# нормализация для перевода в 8-битный формат
	norm_depth_map = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
	# приведение к uint8
	norm_depth_map = np.uint8(norm_depth_map)
	norm_depth_map_pub.publish(cv_bridge.cv2_to_imgmsg(norm_depth_map, "mono8"))
	# }
	print("before return")
	return StereoModeResponse(distance)


def main() -> None:
	rospy.init_node(NODE_NAME)

	stereo_mode_server = rospy.Service(STEREO_MODE_SRV, StereoMode, handle_stereo_mode)

	rospy.spin()


if __name__ == '__main__':
	main()
