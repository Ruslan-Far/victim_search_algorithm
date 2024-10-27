#!/usr/bin/env python3

import cv2
from ultralytics import YOLO
import time

NAME = "yolov10"
TRAIN_HEIGHT = 640
TRAIN_WIDTH = 640
HEIGHT = 480
WIDTH = 744


def process_img(img_rgb, model) -> None:
	img_rgb = cv2.resize(img_rgb, (TRAIN_WIDTH, TRAIN_HEIGHT))

	start = time.perf_counter()
	result = model.predict(img_rgb, conf=0.5, device="cpu") # iou не работает и gpu недоступно
	time_took = time.perf_counter() - start
	print("time_took:", time_took)
	# просто для показа
	cv2.imshow(NAME, cv2.resize(result[0].plot(), (WIDTH, HEIGHT)))
	# cv2.imshow(NAME, result[0].plot())
	cv2.waitKey(0)


def main() -> None:

	# model = YOLO("/home/ruslan/kpfu/magistracy/ml_models/usar_engineer3_ep0-20_yolov10s/best.pt")
	# model = YOLO("/home/ruslan/kpfu/magistracy/ml_models/usar_engineer3_ep0-20_yolov10s/best_openvino_model") # необходимо активировать OpenVINO в терминале: source /opt/intel/openvino/setupvars.sh
	model = YOLO("/home/ruslan/kpfu/magistracy/ml_models/yolov10s_openvino_model")
	# print("model.info()")
	# print(model.info())

	img_path = "/home/ruslan/kpfu/magistracy/test_images/face.jpg"
	# img_path = "/home/ruslan/kpfu/magistracy/test_images/room1412_1_frame0028.jpg"
	# img_path = "/home/ruslan/kpfu/magistracy/test_images/sdv1_45.JPG"
	# img_path = "/home/ruslan/kpfu/magistracy/test_images/sdv1_124.JPG"
	img_rgb = cv2.imread(img_path)
	process_img(img_rgb, model)

	cv2.destroyAllWindows()


if __name__ == '__main__':
	main()
