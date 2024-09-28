#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

# МОЖНО МЕНЯТЬ И ЗАПУСКАТЬ ТОЛЬКО В ДАННОЙ ВЕТКЕ!!! ЗАПУСКАТЬ ТОЛЬКО НА СВОЕМ НОУТБУКЕ!!!

NODE_NAME = "det_stereo_sub_node"
IMG_SUB_TOPIC = "/detected/stereo/left/image_raw"
HEIGHT = 640
WIDTH = 640
WINDOW = "/detected/stereo/left/image_raw"


def img_callback(msg: Image, cv_bridge: CvBridge) -> None:
	img_bgr = cv_bridge.imgmsg_to_cv2(msg)
	img_rgb = img_bgr
	img_rgb = cv2.resize(img_rgb, (WIDTH, HEIGHT))
	cv2.imshow(WINDOW, img_rgb)
	cv2.waitKey(1)


def main() -> None:
	rospy.init_node(NODE_NAME)
	# sample: Image = rospy.wait_for_message(IMG_SUB_TOPIC, Image, timeout = 3.0)
	# if sample is not None:
		# rospy.loginfo(f"Encoding: {sample.encoding}, Resolution: {sample.width, sample.height}")
	cv_bridge: CvBridge = CvBridge()

	rospy.Subscriber(IMG_SUB_TOPIC, Image, lambda msg: img_callback(msg, cv_bridge), queue_size=None)

	rospy.spin()

	cv2.destroyAllWindows()


if __name__ == '__main__':
	main()

# ЗАПУСКАТЬ В ПЕРВУЮ ОЧЕРЕДЬ
# команда для записи rosbag в директории .../scenario_?/rosbags:
# rosbag record /detected/stereo/left/image_raw