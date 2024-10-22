#!/usr/bin/env python3

import cv2
import numpy as np
from openvino.runtime import Core
import openvino as ov
import openvino.properties as props
# from openvino.inference_engine import IENetwork, IECore

NAME = "yolo_openvino"
IMG_PATH = "/home/ruslan/kpfu/magistracy/test_images/sdv1_45.JPG"
TRAIN_HEIGHT = 640
TRAIN_WIDTH = 640

CONFIDENCE_THRESHOLD = 0.5


def draw_detections(frame, result):
	detection_count = 0
	for detection in result[0]:  # цикл по каждому предсказанному объекту
		detection_count += 1
		# print("========================")
		# print(detection)
		# print("=====================")
		confidence = detection[4]  # уверенность детекции
		if confidence > CONFIDENCE_THRESHOLD:  # если уверенность выше порога
			x1, y1, x2, y2 = detection[:4]  # координаты бокса
			class_id = np.argmax(detection[5:])  # индекс класса с наибольшей вероятностью

			# Преобразуем координаты в целые значения для рисования
			x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

			# Рисуем bounding box
			cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

			# Выводим класс объекта и уверенность
			label = f"Class: {class_id}, Conf: {confidence:.2f}"
			cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

	print("detection_count:", detection_count)
	return frame


img_rgb = cv2.imread(IMG_PATH, cv2.IMREAD_COLOR)
img_rgb = cv2.resize(img_rgb, (TRAIN_WIDTH, TRAIN_HEIGHT))
proc_img_rgb = img_rgb.transpose((2, 0, 1))
proc_img_rgb = np.expand_dims(proc_img_rgb, axis=0)
proc_img_rgb = proc_img_rgb.astype(np.float32) / 255.0
# proc_img_rgb = proc_img_rgb.astype(np.float32)

ie = Core()
# model = ie.read_model(model="/home/ruslan/kpfu/magistracy/ml_models/usar_engineer3_ep0-20_yolov8s/best_openvino_model/best.xml")
# model = ie.read_model(model="/home/ruslan/kpfu/magistracy/ml_models/yolov8s_openvino_model/yolov8s.xml")
# model = ie.read_model(model="/home/ruslan/kpfu/magistracy/ml_models/yolov8s_openvino_conv_model/yolov8s.xml")
# model = ie.read_model(model="/home/ruslan/kpfu/magistracy/ml_models/yolov10n_openvino_model/yolov10n.xml")
# model = ie.read_model(model="/home/ruslan/kpfu/magistracy/ml_models/yolov10n_openvino_conv_model/yolov10n.xml")
# model = ie.read_model(model="/home/ruslan/kpfu/magistracy/ml_models/yolov10n_openvino_github/yolov10n.xml")
# model = ie.read_model(model="/home/ruslan/kpfu/magistracy/ml_models/yolov10s_openvino_conv_model/yolov10s.xml")
model = ie.read_model(model="/home/ruslan/kpfu/magistracy/ml_models/yolov10s_openvino_mo/yolov10s.xml")
compiled_model = ie.compile_model(model=model, device_name="CPU")
input_layer = compiled_model.input(0)
output_layer = compiled_model.output(0)

# compiled_model.
print("output_layer", output_layer)
result = compiled_model([proc_img_rgb])[output_layer]
# result = compiled_model([proc_img_rgb])[0]
# result = compiled_model([proc_img_rgb])
print(result.dtype)
print(result.shape)
# print(result)



# ----------------------------------------------------------------------------------------
# core = ov.Core()

# devices = core.available_devices

# for device in devices:
# 	device_name = core.get_property(device, props.device.full_name)
# 	print(f"{device}: {device_name}")
	
# model = core.read_model(model="/home/ruslan/kpfu/magistracy/ml_models/usar_engineer3_ep0-20_yolov8s/best_openvino_model/best.xml")
# compiled_model = core.compile_model(model=model, device_name="CPU")
# print(model.inputs)
# print(model.outputs)
# print(compiled_model.inputs)
# print(compiled_model.outputs)
# print(model.input(0))
# input_layer = model.input(0)
# print(input_layer.any_name)
# print(f"input precision: {input_layer.element_type}")
# print(f"input shape: {input_layer.shape}")
# output_layer = model.output(0)
# # print(output_layer.any_name)
# print(f"output precision: {output_layer.element_type}")
# print(f"output shape: {output_layer.shape}")
# ----------------------------------------------------------------------------------------

cv2.imshow(NAME, draw_detections(img_rgb, result))
cv2.waitKey(0)
cv2.destroyAllWindows()