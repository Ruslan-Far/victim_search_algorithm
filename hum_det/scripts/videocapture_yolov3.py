#!/usr/bin/env python3

import cv2
import numpy as np
import time

# HEIGHT = 1024
# WIDTH = 1280
HEIGHT = 480
WIDTH = 744
WINDOW_ORIG = "original"
WINDOW_YOLOV3 = "yolov3"
CONFIDENCE = 0.25
SCORE_THRESHOLD = 0.25
IOU_THRESHOLD = 0.5
config_path = "/home/ruslan/kpfu/magistracy/ml_models/usar_engineer3_yolov3/cfg/usar_engineer3_yolov3.cfg"
weights_path = "/home/ruslan/kpfu/magistracy/ml_models/usar_engineer3_yolov3/very_very_good_weights/usar_engineer3_yolov3_best_2018.weights"
font_scale = 1
thickness = 2
labels = open("/home/ruslan/kpfu/magistracy/ml_models/usar_engineer3_yolov3/data/usar_engineer3.names").read().strip().split("\n")
colors = np.array([[0, 0, 255], [203, 192, 255], [0, 102, 255], [0, 255, 255]], dtype="uint8")

net = cv2.dnn.readNetFromDarknet(config_path, weights_path)

ln = net.getLayerNames()
ln = [ln[int(i - 1)] for i in net.getUnconnectedOutLayers()]

# получаем видео с камеры
# video=cv2.VideoCapture("rtsp://192.168.0.99:554/av0_0")
video=cv2.VideoCapture(0)
# пока не нажата любая клавиша — выполняем цикл
while cv2.waitKey(1)<0:
	hasFrame, img_rgb = video.read()
	print(hasFrame)
	print(img_rgb)
	img_rgb = cv2.resize(img_rgb, (WIDTH, HEIGHT))
	
	blob = cv2.dnn.blobFromImage(img_rgb, 1/255.0, (416, 416), swapRB=True, crop=False)
	net.setInput(blob)
	start = time.perf_counter()
	layer_outputs = net.forward(ln)
	time_took = time.perf_counter() - start
	print("Time took:", time_took)
	boxes, confidences, class_ids = [], [], []

	# loop over each of the layer outputs
	for output in layer_outputs:
		# loop over each of the object detections
		for detection in output:
			# extract the class id (label) and confidence (as a probability) of
			# the current object detection
			scores = detection[5:]
			class_id = np.argmax(scores)
			confidence = scores[class_id]
			# discard weak predictions by ensuring the detected
			# probability is greater than the minimum probability
			if confidence > CONFIDENCE:
				# scale the bounding box coordinates back relative to the
				# size of the image, keeping in mind that YOLO actually
				# returns the center (x, y)-coordinates of the bounding
				# box followed by the boxes' width and height
				box = detection[:4] * np.array([WIDTH, HEIGHT, WIDTH, HEIGHT])
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
	idxs = cv2.dnn.NMSBoxes(boxes, confidences, SCORE_THRESHOLD, IOU_THRESHOLD)

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

	# cv2.imshow(WINDOW_ORIG, img_rgb)
	cv2.imshow(WINDOW_YOLOV3, img_rgb)
	cv2.waitKey(1)

cv2.destroyAllWindows()

