#!/usr/bin/env python3

import rospy
from hum_det.srv import *
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image

NODE_NAME = "stereo_node"

NORM_DEPTH_MAP_TOPIC = "/depth_map_roi"

STEREO_MODE_SRV = "/stereo_mode"

norm_depth_map_pub = rospy.Publisher(NORM_DEPTH_MAP_TOPIC, Image, queue_size=1)

cv_bridge = CvBridge()


def run_norm_depth_map_pub(req, depth_map):
	max_distance = req.disp_img.f * req.disp_img.T / req.disp_img.min_disparity # req.disp_img.min_disparity должен быть >= 1
	norm_depth_map = (255 * depth_map / max_distance).astype(np.uint8)
	norm_depth_map_pub.publish(cv_bridge.cv2_to_imgmsg(norm_depth_map, "mono8"))


def handle_stereo_mode(req):
	rospy.loginfo("===handle_stereo_mode===")
	rospy.loginfo(f"req.bbox.x: {req.bbox.x}")
	rospy.loginfo(f"req.bbox.y: {req.bbox.y}")
	rospy.loginfo(f"req.bbox.w: {req.bbox.w}")
	rospy.loginfo(f"req.bbox.h: {req.bbox.h}")
	rospy.loginfo(f"req.disp_img.f: {req.disp_img.f}")
	rospy.loginfo(f"req.disp_img.T: {req.disp_img.T}")
	rospy.loginfo(f"req.disp_img.image.width: {req.disp_img.image.width}")
	rospy.loginfo(f"req.disp_img.image.height: {req.disp_img.image.height}")
	rospy.loginfo(f"req.disp_img.min_disparity: {req.disp_img.min_disparity}")
	rospy.loginfo(f"req.disp_img.max_disparity: {req.disp_img.max_disparity}")

	disp_img_image = cv_bridge.imgmsg_to_cv2(req.disp_img.image, desired_encoding="32FC1") # используется для подсчета расстояния
	roi = disp_img_image[req.bbox.y:req.bbox.y+req.bbox.h, req.bbox.x:req.bbox.x+req.bbox.w]
	depth_map = np.where(roi >= req.disp_img.min_disparity, req.disp_img.f * req.disp_img.T / roi, 0)
	valid_pixels = depth_map[depth_map > 0]
	if valid_pixels.size > 0:
		median_distance = np.median(valid_pixels) # медиана
		mean_distance = np.mean(valid_pixels) # среднее значение
		rospy.loginfo(f"медианная дистанция: {median_distance}")
		rospy.loginfo(f"средняя дистанция: {mean_distance}")
		distance = mean_distance
	else:
		rospy.loginfo("нет валидных пикселей в roi!")
		distance = 0
	rospy.loginfo(f"valid_pixels.size: {valid_pixels.size}")
	run_norm_depth_map_pub(req, depth_map) # если будут наблюдаться проблемы с производительностью, то можно закомментировать
	return StereoModeResponse(distance)


def main() -> None:
	rospy.init_node(NODE_NAME)

	stereo_mode_server = rospy.Service(STEREO_MODE_SRV, StereoMode, handle_stereo_mode)

	rospy.spin()


if __name__ == '__main__':
	main()
