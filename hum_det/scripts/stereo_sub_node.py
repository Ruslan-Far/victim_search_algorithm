#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

NODE_NAME = "stereo_sub_node"
IMG_SUB_TOPIC = "/stereo/left/image_raw"
# IMG_SUB_TOPIC = "/usb_cam/image_raw"
HEIGHT = 640
WIDTH = 640
WINDOW = "/stereo/left/image_raw"


def img_callback(msg: Image, cv_bridge: CvBridge) -> None:
	# иначе будет серое байеризованное
	img_bgr = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
	# иначе будет bgr (если запускать на Инженере (иначе - закомментить)). И намного хуже будет распознавать нейронка
	img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
	# если запускать на своем ноутбуке (иначе - закомментить)
	# img_rgb = img_bgr
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
