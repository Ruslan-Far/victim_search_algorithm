#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import time

ROS_NODE_NAME = "first_sub"
# ROS_IMAGE_TOPIC = "/pylon_camera_node/image_raw"
ROS_IMAGE_TOPIC = "/usb_cam/image_raw"
HEIGHT = 1024
WIDTH = 1280
WINDOW_ORIG = "original"
WINDOW_YOLOV3 = "yolov3"
CONFIDENCE = 0.5
SCORE_THRESHOLD = 0.5
IOU_THRESHOLD = 0.5
config_path = "/home/ruslan/kpfu/magistracy/ml_models/yolov3/yolo-obj.cfg"
weights_path = "/home/ruslan/kpfu/magistracy/ml_models/yolov3/yolo-obj.weights"
font_scale = 1
thickness = 1
labels = open("/home/ruslan/kpfu/magistracy/ml_models/yolov3/obj.names").read().strip().split("\n")
colors = np.random.randint(0, 255, size=(len(labels), 3), dtype="uint8")

net = cv2.dnn.readNetFromDarknet(config_path, weights_path)

ln = net.getLayerNames()
ln = [ln[int(i - 1)] for i in net.getUnconnectedOutLayers()]


def image_callback(msg: Image, cv_bridge: CvBridge) -> None:
	img_bgr = cv_bridge.imgmsg_to_cv2(msg)
	img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
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

	font_scale = 1
	thickness = 1

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


def main() -> None:
	rospy.init_node(ROS_NODE_NAME)
	sample: Image = rospy.wait_for_message(ROS_IMAGE_TOPIC, Image, timeout = 3.0)
	if sample is not None:
		rospy.loginfo(f"Encoding: {sample.encoding}, Resolution: {sample.width, sample.height}")
	cv_bridge: CvBridge = CvBridge()

	rospy.Subscriber(ROS_IMAGE_TOPIC, Image, lambda msg: image_callback(msg, cv_bridge), queue_size = None)

	rospy.spin()

	# Close down the video stream when done
	# cv2.destroyAllWindows()


if __name__ == '__main__':
	main()
