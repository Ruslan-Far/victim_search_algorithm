#!/usr/bin/env python3

import cv2
import numpy as np
import time

NAME = "yolov3"
# IMG_PATH = "/home/ruslan/kpfu/magistracy/test_images/face.jpg"
# IMG_PATH = "/home/ruslan/kpfu/magistracy/test_images/room1412_1_frame0028.jpg"
# IMG_PATH = "/home/ruslan/kpfu/magistracy/test_images/sdv1_124.JPG"
IMG_PATH = "/home/ruslan/kpfu/magistracy/test_images/sdv1_45.JPG"
TRAIN_HEIGHT = 608
TRAIN_WIDTH = 608
HEIGHT = 480
WIDTH = 744

CONFIDENCE_THRESHOLD = 0
IOU_THRESHOLD = 1

config_path = "/home/ruslan/kpfu/magistracy/ml_models/usar_engineer3_yolov3/cfg/usar_engineer3_yolov3.cfg"
weights_path = "/home/ruslan/kpfu/magistracy/ml_models/usar_engineer3_yolov3/very_very_good_weights/usar_engineer3_yolov3_best_2018.weights"

font_scale = 1
thickness = 2

labels = open("/home/ruslan/kpfu/magistracy/ml_models/usar_engineer3_yolov3/data/usar_engineer3.names").read().strip().split("\n")
colors = np.array([[0, 0, 255], [203, 192, 255], [0, 102, 255], [0, 255, 255]], dtype="uint8")

net = cv2.dnn.readNetFromDarknet(config_path, weights_path)

ln = net.getLayerNames()
# print(ln)
ln = [ln[int(i - 1)] for i in net.getUnconnectedOutLayers()]
# print("===========================================================")
# print(net.getUnconnectedOutLayers())
# print("===========================================================")
# print(ln)


def main() -> None:
	img_rgb = cv2.imread(IMG_PATH, cv2.IMREAD_COLOR)
	img_rgb = cv2.resize(img_rgb, (TRAIN_WIDTH, TRAIN_HEIGHT))

	blob = cv2.dnn.blobFromImage(img_rgb, 1/255.0, swapRB=False, crop=False)
	net.setInput(blob)
	start = time.perf_counter()
	layer_outputs = net.forward(ln)
	time_took = time.perf_counter() - start
	print("time_took:", time_took)
	boxes, confidences, class_ids = [], [], []

	print("===========================================")
	print(layer_outputs)
	output_count = 0
	detection_count = 0
	conf_count = 0
	# loop over each of the layer outputs
	for output in layer_outputs:
		print("???????????????????????????????????????????")
		print(output)
		print("???????????????????????????????????????????")
		output_count += 1
		# loop over each of the object detections
		for detection in output:
			# print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
			# print(detection)
			# print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
			detection_count += 1
			# extract the class id (label) and confidence (as a probability) of
			# the current object detection
			scores = detection[5:]
			class_id = np.argmax(scores)
			confidence = scores[class_id]
			# discard weak predictions by ensuring the detected
			# probability is greater than the minimum probability
			if confidence > CONFIDENCE_THRESHOLD:
				# scale the bounding box coordinates back relative to the
				# size of the image, keeping in mind that YOLO actually
				# returns the center (x, y)-coordinates of the bounding
				# box followed by the boxes' width and height
				print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
				print(detection)
				print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
				conf_count += 1

				box = detection[:4] * np.array([TRAIN_WIDTH, TRAIN_HEIGHT, TRAIN_WIDTH, TRAIN_HEIGHT])
				(centerX, centerY, width, height) = box.astype("int")

				# use the center (x, y)-coordinates to derive the top and
				# and left corner of the bounding box
				x = int(centerX - (width / 2))
				y = int(centerY - (height / 2))

				# update our list of bounding box coordinates, confidences,
				# and class IDs
				boxes.append([x, y, int(width), int(height)])
				confidences.append(float(confidence))
				class_ids.append(class_id)

	# perform the non maximum suppression given the scores defined before
	idxs = cv2.dnn.NMSBoxes(boxes, confidences, CONFIDENCE_THRESHOLD, IOU_THRESHOLD)

	# ensure at least one detection exists
	if len(idxs) > 0:
		# loop over the indexes we are keeping
		for i in idxs.flatten():
			# extract the bounding box coordinates
			x, y = boxes[i][0], boxes[i][1]
			w, h = boxes[i][2], boxes[i][3]
			# draw a bounding box rectangle and label on the image
			color = [int(c) for c in colors[class_ids[i]]]
			cv2.rectangle(img_rgb, (x, y), (x + w, y + h), color=color, thickness=thickness)
			text = f"{labels[class_ids[i]]}: {confidences[i]:.2f}"
			# calculate text width & height to draw the transparent boxes as background of the text
			(text_width, text_height) = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, fontScale=font_scale, thickness=thickness)[0]
			text_offset_x = x
			text_offset_y = y - 5
			box_coords = ((text_offset_x, text_offset_y), (text_offset_x + text_width + 2, text_offset_y - text_height))
			overlay = img_rgb.copy()
			cv2.rectangle(overlay, box_coords[0], box_coords[1], color=color, thickness=cv2.FILLED)
			# add opacity (transparency to the box)
			img_rgb = cv2.addWeighted(overlay, 0.6, img_rgb, 0.4, 0)
			# now put the text (label: confidence %)
			cv2.putText(img_rgb, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
				fontScale=font_scale, color=(0, 0, 0), thickness=thickness)

	print("output_count", output_count)
	print("detection_count", detection_count)
	print("conf_count", conf_count)
	print("len(idxs)", len(idxs))

	# img_rgb = cv2.resize(img_rgb, (WIDTH, HEIGHT))
	# просто для показа
	cv2.imshow(NAME, img_rgb)
	cv2.waitKey(0)
	cv2.destroyAllWindows()


if __name__ == '__main__':
	main()
