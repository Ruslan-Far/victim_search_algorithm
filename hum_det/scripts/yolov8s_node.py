#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import time

NODE_NAME = "yolov8s_node"
IMG_SUB_TOPIC = "/usb_cam_node/image_raw"
TRAIN_HEIGHT = 640
TRAIN_WIDTH = 640
HEIGHT = 480
WIDTH = 744
WINDOW_ORIG = "original"
FREQ = 1
img_callback_count = 0

time_took_sum = 0
time_took_count = 0


# на основе N итераций посчитать, сколько времени в среднем занимает вывод нейросети за одну итерацию
def print_time_took_mean_sum(time_took):
	global time_took_sum
	global time_took_count

	time_took_sum += time_took
	time_took_count += 1
	if time_took_count == 10: # N == 10
		print("time_took_mean_sum:", time_took_sum / time_took_count)
		time_took_sum = 0
		time_took_count = 0


def img_callback(msg: Image, cv_bridge: CvBridge, model) -> None:
	global img_callback_count

	# иначе будет серое байеризованное
	img_bgr = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
	# иначе будет bgr (если запускать на Инженере (иначе - закомментить)). И намного хуже будет распознавать нейронка
	# img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
	# если запускать на своем ноутбуке (иначе - закомментить)
	img_rgb = img_bgr
	img_rgb = cv2.resize(img_rgb, (TRAIN_WIDTH, TRAIN_HEIGHT))

	if img_callback_count == 0:
		start = time.perf_counter()
		result = model.predict(img_rgb)
		time_took = time.perf_counter() - start
		print("time_took:", time_took)
		print_time_took_mean_sum(time_took)
		# просто для показа
		cv2.imshow(NODE_NAME, cv2.resize(result[0].plot(), (WIDTH, HEIGHT)))
		# cv2.imshow(NODE_NAME, result[0].plot())
		cv2.waitKey(1)
	img_callback_count += 1
	if img_callback_count == FREQ:
		img_callback_count = 0


def main() -> None:
	rospy.init_node(NODE_NAME)
	sample: Image = rospy.wait_for_message(IMG_SUB_TOPIC, Image, timeout=3.0)
	if sample is not None:
		rospy.loginfo(f"Encoding: {sample.encoding}, Resolution: {sample.width, sample.height}")
	cv_bridge: CvBridge = CvBridge()

	# model = YOLO("/home/ruslan/kpfu/magistracy/ml_models/usar_ep20-60_yolov8s/best.pt") # bad
	# model = YOLO("/home/ruslan/kpfu/magistracy/ml_models/usar_engineer_ep0-20_yolov8s/best.pt")
	# model = YOLO("/home/ruslan/kpfu/magistracy/ml_models/usar_engineer2_ep0-20_yolov8s/best.pt") # bad
	model = YOLO("/home/ruslan/kpfu/magistracy/ml_models/usar_engineer3_ep0-20_yolov8s/best.pt")
	# model = YOLO("/home/ruslan/kpfu/magistracy/ml_models/yolov8s.pt")
	# model = YOLO("/home/ruslan/kpfu/magistracy/ml_models/usar_ep0-20_yolov8x/best.pt") # bad + speed bad
	# print(model.info())

	rospy.Subscriber(IMG_SUB_TOPIC, Image, lambda msg: img_callback(msg, cv_bridge, model), queue_size=None)

	rospy.spin()

	cv2.destroyAllWindows()


if __name__ == '__main__':
	main()
