#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from openvino.runtime import Core


def draw_detections(frame, result, conf_threshold=0.5):
	"""
	Функция для обработки результатов инференса и отображения bounding boxes на изображении.
	frame: изображение (кадр)
	result: результаты инференса (предполагаем, что это numpy массив)
	conf_threshold: порог уверенности для отображения боксов
	"""
	detection_count = 0
	for detection in result[0]:  # цикл по каждому предсказанному объекту
		detection_count += 1
		print("========================")
		print(detection)
		print("=====================")
		confidence = detection[4]  # уверенность детекции
		if confidence > conf_threshold:  # если уверенность выше порога
			x1, y1, x2, y2 = detection[:4]  # координаты бокса
			class_id = np.argmax(detection[5:])  # индекс класса с наибольшей вероятностью

			# Преобразуем координаты в целые значения для рисования
			x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
			# print(x1)
			# print(y1)
			# print(x2)
			# print(y2)

			# frame = cv2.UMat(frame).get()
			# Рисуем bounding box
			# cv2.rectangle(frame, (x1, y1), (x1 + x2, y1 + y2), (0, 255, 0), 2)
			cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
			# cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
			# cv2.rectangle(frame, (x1, y1), (x2, y2), (int(0), int(255), int(0)), thickness=2)
			# cv2.rectangle(frame, x1, y1, x2, y2, (0, 255, 0), 2)

			# Выводим класс объекта и уверенность
			label = f"Class: {class_id}, Conf: {confidence:.2f}"
			cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

	print("detection_count:", detection_count)
	return frame


# def draw_detections(frame, result, conf_threshold=0.5):
#     """
#     Функция для обработки результатов инференса и отображения bounding boxes на изображении.
#     frame: исходное изображение (кадр)
#     result: результаты инференса (массив numpy)
#     conf_threshold: порог уверенности для отображения боксов
#     """
#     height, width = frame.shape[:2]
    
#     # result - это выход модели YOLO, содержащий данные о предсказанных объектах
#     for detection in result[0]:  # цикл по каждому предсказанному объекту
#         confidence = detection[4]  # уверенность детекции
#         if confidence >= conf_threshold:  # если уверенность выше порога
#             # Получаем координаты бокса и масштабируем их обратно к размеру изображения
#             x1, y1, x2, y2 = detection[:4]
#             x1 = int(x1 * width)
#             y1 = int(y1 * height)
#             x2 = int(x2 * width)
#             y2 = int(y2 * height)
            
#             # Получаем индекс класса
#             class_id = np.argmax(detection[5:])  # индекс класса с наибольшей вероятностью
            
#             # Рисуем bounding box
#             cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
#             # Выводим класс объекта и уверенность
#             label = f"Class: {class_id}, Conf: {confidence:.2f}"
#             cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

#     return frame


def image_callback(msg: Image, cv_bridge: CvBridge, compiled_model, output_layer) -> None:
	# иначе будет серое байеризованное
	img_bgr = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
	# иначе будет bgr (если запускать на Инженере (иначе - закомментить)). И намного хуже будет распознавать нейронка
	# img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
	# если запускать на своем ноутбуке (иначе - закомментить)
	img_rgb = img_bgr
	img_rgb = cv2.resize(img_rgb, (640, 640))
	proc_img_rgb = img_rgb.transpose((2, 0, 1))
	proc_img_rgb = np.expand_dims(proc_img_rgb, axis=0)
	proc_img_rgb = proc_img_rgb.astype(np.float32) / 255.0

	result = compiled_model([proc_img_rgb])[output_layer]
	# result = result.transpose((1, 2, 0))

	print(result)

	# publisher = rospy.Publisher("/newstereo/left/image_raw", Image, queue_size = 10)
	# image = cv_bridge.cv2_to_imgmsg(result, "rgb8")
	# publisher.publish(image)
	print(result.dtype)
	print(result.shape)

	# cv2.imshow("yolo_openvino_node", result)
	cv2.imshow("yolo_openvino_node", draw_detections(img_rgb, result))
	# print(draw_detections(img_rgb, result))

	# cv2.imshow("yolo_openvino_node", img_rgb)
	cv2.waitKey(1)


def main() -> None:
	rospy.init_node("yolo_openvino_node")
	sample: Image = rospy.wait_for_message("/usb_cam_node/image_raw", Image, timeout = 3.0)
	if sample is not None:
		rospy.loginfo(f"Encoding: {sample.encoding}, Resolution: {sample.width, sample.height}")
	cv_bridge: CvBridge = CvBridge()

	ie = Core()
	model = ie.read_model(model="/home/ruslan/kpfu/magistracy/ml_models/usar_engineer3_ep0-20_yolov8s/best_openvino_model/best.xml")
	compiled_model = ie.compile_model(model=model, device_name="CPU")
	input_layer = compiled_model.input(0)
	output_layer = compiled_model.output(0)

	rospy.Subscriber("/usb_cam_node/image_raw", Image, lambda msg: image_callback(msg, cv_bridge, compiled_model, output_layer), queue_size=None)

	rospy.spin()

	cv2.destroyAllWindows()


if __name__ == '__main__':
	main()
