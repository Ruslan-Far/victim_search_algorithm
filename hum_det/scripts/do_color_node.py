#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from typing import Final
from hum_det.msg import DetectionStatus

# НЕОБХОДИМ для экспериментов. Будет прослушивать камеру робота и преобразовывать ее видеопоток в небайеризированный rgb формат
ROS_NODE_NAME: Final[str] = "do_color_node"
# ROS_IMAGE_TOPIC: Final[str] = "/usb_cam/image_raw"
ROS_IMAGE_TOPIC: Final[str] = "/stereo/left/image_raw"
WIDTH: Final[int] = 744
HEIGHT: Final[int] = 480
FREQ = 1 # ВАЖНО!!! Иначе (если прослушивать топики камеры инженера удаленно по сети) будет очень сильно зависать и будет очень мало разнообразных изображений!
img_callback_count = 0
IS_ON = True # по умолчанию алгоритм работать не будет. Как из GUI придет сигнал о начале работы, он начнет работу (True)
DET_STATUS_SUB_TOPIC = "/gui"

def reset_fields():
	global img_callback_count

	img_callback_count = 0
	print("reset_fields")


# получаем статус режима "human_detection" в gui и переключаем
def det_status_callback(msg):
	global IS_ON

	if IS_ON != msg.is_on:
		IS_ON = msg.is_on
		if IS_ON:
			reset_fields()


def image_callback(msg: Image, cv_bridge: CvBridge) -> None:
	global img_callback_count

	if not IS_ON:
		return
	img_bgr = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
	# img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
	img_rgb = img_bgr
	img_rgb = cv2.resize(img_rgb, (WIDTH, HEIGHT))

	if img_callback_count == 0:
		publisher = rospy.Publisher("/newstereo/left/image_raw", Image, queue_size=1)
		image = cv_bridge.cv2_to_imgmsg(img_rgb, "rgb8")
		publisher.publish(image)
	img_callback_count += 1
	if img_callback_count == FREQ:
		img_callback_count = 0


def main() -> None:
	rospy.init_node(ROS_NODE_NAME)

	# sample: Image = rospy.wait_for_message(ROS_IMAGE_TOPIC, Image, timeout = 33.0)

	# if sample is not None:
		# rospy.loginfo(f"Encoding: {sample.encoding}, Resolution: {sample.width, sample.height}")

	cv_bridge: CvBridge = CvBridge()

	rospy.Subscriber(ROS_IMAGE_TOPIC, Image, lambda msg: image_callback(msg, cv_bridge), queue_size = None)
	rospy.Subscriber(DET_STATUS_SUB_TOPIC, DetectionStatus, lambda msg: det_status_callback(msg), queue_size=None)

	rospy.spin()


if __name__ == '__main__':
	main()

# ЗАПУСКАТЬ В ПЕРВУЮ ОЧЕРЕДЬ
# команда для записи rosbag в директории .../scenario_?/rosbags:
# rosbag record /newstereo/left/image_raw