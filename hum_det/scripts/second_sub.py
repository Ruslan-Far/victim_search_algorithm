#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

ROS_NODE_NAME = "second_sub"
ROS_IMAGE_TOPIC = "/newstereo/left/image_raw"
# ROS_IMAGE_TOPIC = "/stereo/left/image_raw"
# ROS_IMAGE_TOPIC = "/pylon_camera_node/image_raw"
# ROS_IMAGE_TOPIC = "/usb_cam/image_raw"
# ROS_IMAGE_TOPIC = "/center/image_raw"
HEIGHT = 512
WIDTH = 640
WINDOW_ORIG = "original"
WINDOW_YOLOV8 = "yolov8"
FREQ = 20

count = 0

def image_callback(msg: Image, cv_bridge: CvBridge, model) -> None:
	global count
	img_bgr = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
	img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
	img_rgb = cv2.resize(img_rgb, (WIDTH, HEIGHT))

	# preds = model.predict(img_rgb, stream=True)
	# cv2.imshow(WINDOW_ORIG, img_rgb)
	# cv2.imshow(WINDOW_YOLOV8S, Image.fromarray(preds[0].plot()[..., ::-1]))

	if count % FREQ == 0:
		preds = model.predict(img_rgb)
		cv2.imshow(WINDOW_YOLOV8, preds[0].plot())
		cv2.waitKey(1)
	count += 1
	if count == FREQ:
		count = 0


def main() -> None:
	rospy.init_node(ROS_NODE_NAME)
	sample: Image = rospy.wait_for_message(ROS_IMAGE_TOPIC, Image, timeout = 3.0)
	if sample is not None:
		rospy.loginfo(f"Encoding: {sample.encoding}, Resolution: {sample.width, sample.height}")
	cv_bridge: CvBridge = CvBridge()

# ------------------------------------------------------------------
	model = YOLO("/home/ruslan/kpfu/magistracy/ml_models/usar_ep20-60_yolov8s/best.pt")
	# model = YOLO("/home/ruslan/kpfu/magistracy/ml_models/yolov8s.pt")
	# model = YOLO("/home/ruslan/kpfu/magistracy/ml_models/usar_ep0-20_yolov8x/best.pt")
	# print(model.info())
	# preds = model.predict("../test/human_27.jpg")
	# for pred in preds:
	# 	print(pred.names)
	# for pred in preds:
	# 	print(pred.boxes)
	# for pred in preds:
	# 	pred.save(filename="../test/preds/usar_ep20-60_yolov8s/human_27.jpg")
# ------------------------------------------------------------------

	# rospy.Subscriber(ROS_IMAGE_TOPIC, Image, lambda msg: image_callback(msg, cv_bridge, model), queue_size = None)
	rospy.Subscriber(ROS_IMAGE_TOPIC, Image, lambda msg: image_callback(msg, cv_bridge, model), queue_size = 10000)

	rospy.spin()

	cv2.destroyAllWindows()


if __name__ == '__main__':
	main()
