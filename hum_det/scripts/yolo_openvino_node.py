#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from openvino.runtime import Core


def image_callback(msg: Image, cv_bridge: CvBridge, compiled_model, output_layer) -> None:
	# иначе будет серое байеризованное
	img_bgr = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
	# иначе будет bgr (если запускать на Инженере (иначе - закомментить)). И намного хуже будет распознавать нейронка
	img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
	# если запускать на своем ноутбуке (иначе - закомментить)
	# img_rgb = img_bgr
	img_rgb = cv2.resize(img_rgb, (640, 640))
	img_rgb = img_rgb.transpose((2, 0, 1))
	img_rgb = np.expand_dims(img_rgb, axis=0)
	img_rgb = img_rgb.astype(np.float32) / 255.0

	result = compiled_model([img_rgb])[output_layer]
	# result = result.transpose((1, 2, 0))

	# print(result)

	# publisher = rospy.Publisher("/newstereo/left/image_raw", Image, queue_size = 10)
	# image = cv_bridge.cv2_to_imgmsg(result, "rgb8")
	# publisher.publish(image)
	print(result.dtype)
	print(result.shape)

	# cv2.imshow("yolo_openvino_node", result)
	cv2.waitKey(1)


def main() -> None:
	rospy.init_node("yolo_openvino_node")
	sample: Image = rospy.wait_for_message("/stereo/left/image_raw", Image, timeout = 3.0)
	if sample is not None:
		rospy.loginfo(f"Encoding: {sample.encoding}, Resolution: {sample.width, sample.height}")
	cv_bridge: CvBridge = CvBridge()

	# model = YOLO("/home/ruslan/kpfu/magistracy/ml_models/usar_ep20-60_yolov8s/best.pt") # bad
	# model = YOLO("/home/ruslan/kpfu/magistracy/ml_models/usar_engineer_ep0-20_yolov8s/best.pt")
	# model = YOLO("/home/ruslan/kpfu/magistracy/ml_models/usar_engineer2_ep0-20_yolov8s/best.pt") # bad
	# model = YOLO("/home/ruslan/kpfu/magistracy/ml_models/usar_engineer3_ep0-20_yolov8s/best.pt")
	# model = YOLO("/home/ruslan/kpfu/magistracy/ml_models/yolov8s.pt")
	# model = YOLO("/home/ruslan/kpfu/magistracy/ml_models/usar_ep0-20_yolov8x/best.pt") # bad + speed bad
	# print(model.info())

	ie = Core()
	model = ie.read_model(model="/home/lirs/ruslan/kpfu/magistracy/ml_models/usar_engineer3_ep0-20_yolov8s/best_openvino/best.xml")
	compiled_model = ie.compile_model(model=model, device_name="CPU")
	input_layer = compiled_model.input(0)
	output_layer = compiled_model.output(0)

	rospy.Subscriber("/stereo/left/image_raw", Image, lambda msg: image_callback(msg, cv_bridge, compiled_model, output_layer), queue_size=None)

	rospy.spin()

	cv2.destroyAllWindows()


if __name__ == '__main__':
	main()
