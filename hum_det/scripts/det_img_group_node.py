#!/usr/bin/env python3

import rospy
from hum_det.msg import DetArray
from hum_det.srv import *
from cv_bridge import CvBridge
import cv2
from det_class import DetClass

NODE_NAME = "det_img_group_node"

DET_ARRAY_TOPIC = "/det_array"
GOAL_DET_TOPIC = "/goal_det"

DET_GROUP_MODE_SWITCH_SRV = "det_group_mode_switch"

MAX_FRAMES = 10 # в будущем заменить на более маленькое число
MIN_DETECTION_RATE = 0.7 # в будущем заменить на более маленькое число
MIN_IOU = 0.8

is_on = False
cv_bridge = CvBridge()

goal_det_pub = rospy.Publisher(GOAL_DET_TOPIC, DetArray, queue_size=1)


def reset_fields():
	global detection_history

	detection_history = []
	print("reset_fields")


def fit_detection_history(msg_dets): # to do
	global detection_history

	old_len_detection_history = len(detection_history) # чтобы не делать лишние последние итерации в цикле
	for msg_det in msg_dets:
		for i in range(old_len_detection_history):
			last_non_none_det = next((det for det in reversed(detection_history[i]) if det is not None), None)
			if not last_non_none_det.is_processed and last_non_none_det.msg_det.class_id == 3: # to do
				detection_history[i].append(DetClass(msg_det, True))
				break
		detection_history.append([DetClass(msg_det, True)])
	# добавление элементов None туда, где последний элемент есть None или is_processed = False. Также, где is_processed = True, сделать снова False, но не добавлять None
	for history in detection_history:
		if history[-1] is None or not history[-1].is_processed:
			history[-1].append(None)
		else:
			history[-1].is_processed = False
		# ограничиваем размер каждого history
		if len(history) > MAX_FRAMES:
			history.pop(0)
			# если history состоит полностью из всех элементов None
			if next((det for det in history if det is not None), None) is None:
				detection_history.remove(history)


def get_best_class_obj(): # to do
	global detection_history

	return ? # to do


def run_goal_det_pub(det, img_rgb):
	msg = DetArray()

	msg.dets = det
	msg.img = cv_bridge.cv2_to_imgmsg(img_rgb, "bgr8")
	goal_det_pub.publish(msg)


def det_array_callback(msg): # to do
	global is_on
	global detection_history

	if not is_on:
		return
	# потом обязательно удалить {
	# иначе будет серое байеризованное
	img_bgr = cv_bridge.imgmsg_to_cv2(msg.img, desired_encoding="bgr8")
	# иначе будет bgr (если запускать на Инженере (иначе - закомментить)). И намного хуже будет распознавать нейронка
	# img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
	# если запускать на своем ноутбуке (иначе - закомментить)
	img_rgb = img_bgr
	cv2.imshow(NODE_NAME, img_rgb)
	cv2.waitKey(1)
	# }

	# detections = msg.dets
	fit_detection_history(msg.dets)

	run_goal_det_pub(?, img_rgb) # to do


# переключаем режим "group_human_detection" (может в будущем переименовать) во вкл/выкл состояние
def handle_det_group_mode_switch(req):
	global is_on

	is_on = req.is_on
	if is_on:
		reset_fields()
	return DetModeSwitchResponse(0) # операция прошла успешно


def main() -> None:
	rospy.init_node(NODE_NAME)

	det_array_sub = rospy.Subscriber(DET_ARRAY_TOPIC, DetArray, det_array_callback, queue_size=1)

	det_group_mode_switch_server = rospy.Service(DET_GROUP_MODE_SWITCH_SRV, DetModeSwitch, handle_det_group_mode_switch)

	rospy.spin()


if __name__ == '__main__':
	main()
