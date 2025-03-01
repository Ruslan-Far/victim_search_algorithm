#!/usr/bin/env python3

import rospy
from hum_det.msg import DetArray, Det
from hum_det.srv import *
from cv_bridge import CvBridge
import cv2
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
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
detection_history = []

goal_det_pub = rospy.Publisher(GOAL_DET_TOPIC, DetArray, queue_size=1)


def reset_fields():
	global detection_history

	detection_history = []
	print("reset_fields")


def calculate_iou(box1, box2):
	# координаты правого нижнего угла
	x1_max, y1_max = box1[0] + box1[2], box1[1] + box1[3]
	x2_max, y2_max = box2[0] + box2[2], box2[1] + box2[3]
	# координаты пересечения
	x_inter_min = max(box1[0], box2[0])
	y_inter_min = max(box1[1], box2[1])
	x_inter_max = min(x1_max, x2_max)
	y_inter_max = min(y1_max, y2_max)
	# вычисление площади пересечения
	inter_width = max(0, x_inter_max - x_inter_min)
	inter_height = max(0, y_inter_max - y_inter_min)
	inter_area = inter_width * inter_height
	# вычисление площади каждого прямоугольника
	box1_area = box1[2] * box1[3]
	box2_area = box2[2] * box2[3]
	# вычисление IoU
	union_area = box1_area + box2_area - inter_area
	iou = inter_area / union_area if union_area > 0 else 0
	return iou


def fit_detection_history(msg_dets):
	global detection_history

	print("===fit_detection_history===")
	old_len_detection_history = len(detection_history) # чтобы не делать лишние последние итерации в цикле
	print("old_len_detection_history =", old_len_detection_history)
	for msg_det in msg_dets:
		for i in range(old_len_detection_history):
			last_non_none_det = next((det for det in reversed(detection_history[i]) if det is not None), None)
			if not last_non_none_det.is_processed and calculate_iou([last_non_none_det.msg_det.bbox.x, last_non_none_det.msg_det.bbox.y,
															last_non_none_det.msg_det.bbox.w, last_non_none_det.msg_det.bbox.h],
															[msg_det.bbox.x, msg_det.bbox.y, msg_det.bbox.w, msg_det.bbox.h]) >= MIN_IOU:
				detection_history[i].append(DetClass(msg_det, True))
				break
		detection_history.append([DetClass(msg_det, True)])
	# добавление элементов None туда, где последний элемент есть None или is_processed = False. Также, где is_processed = True, сделать снова False, но не добавлять None
	lst_for_clean = []
	for history in detection_history:
		if history[-1] is None or not history[-1].is_processed:
			history.append(None)
		else:
			history[-1].is_processed = False
		# ограничиваем размер каждого history
		if len(history) > MAX_FRAMES:
			history.pop(0)
			# если history состоит полностью из всех элементов None
			if next((det for det in history if det is not None), None) is None:
				lst_for_clean.append(history)
	# удаление тех history, которые полностью состоят из элементов None
	for history in lst_for_clean:
		detection_history.remove(history)


def get_goal_det():
	global detection_history

	print("===get_goal_det===")
	max_history_id = -1
	max_detection_rate = MIN_DETECTION_RATE
	max_class_id = -1
	max_avg_conf = -1
	for hist_id, history in enumerate(detection_history):
		# если данные еще не накопились в полном объеме
		if len(history) != MAX_FRAMES:
			print("если данные еще не накопились в полном объеме")
			continue
		# ----------------------------------------------------------------- step 1
		print("----------------------------------------------------------------- step 1")
		tmp_rates = [0, 0, 0, 0] # foot, head, hand, person
		for det in history:
			if det is None:
				continue
			tmp_rates[det.msg_det.class_id] += 1
		for cls_id in range(len(tmp_rates)):
			tmp_rates[cls_id] /= len(history)
		tmp_rate = max(tmp_rates)
		if tmp_rate < max_detection_rate:
			print("коэффициент обнаружения объекта класса оказался меньше необходимого порога {")
			print("tmp_rate", tmp_rate)
			print("tmp_class_id", tmp_rates.index(tmp_rate))
			print("}")
			continue
		# ----------------------------------------------------------------- step 2
		print("----------------------------------------------------------------- step 2")
		tmp_class_id = tmp_rates.index(tmp_rate)
		count = 0
		tmp_avg_conf = 0
		for det in history:
			if det is None or det.msg_det.class_id != tmp_class_id:
				continue
			count += 1
			tmp_avg_conf += det.msg_det.confidence
		tmp_avg_conf /= count
		print("tmp_class_id", tmp_class_id)
		print("count", count)
		print("tmp_avg_conf", tmp_avg_conf)
		if tmp_rate == max_detection_rate:
			print("tmp_rate == max_detection_rate")
			if tmp_avg_conf <= max_avg_conf:
				print("tmp_avg_conf <= max_avg_conf")
				continue
		max_history_id = hist_id
		max_detection_rate = tmp_rate
		max_class_id = tmp_class_id
		max_avg_conf = tmp_avg_conf
		print("max_history_id", max_history_id)
		print("max_detection_rate", max_detection_rate)
		print("max_class_id", max_class_id)
		print("max_avg_conf", max_avg_conf)
	# если не накопилось достаточно данных или все коэффициенты обнаружений объектов классов оказались меньше необходимого порога
	if max_class_id == -1:
		print("если не накопилось достаточно данных или все коэффициенты обнаружений объектов классов оказались меньше необходимого порога")
		return None
	# --------------------------------------------------------------------- step 3
	print("----------------------------------------------------------------- step 3")
	# иначе сформировать наилучшее получившееся обнаружение объекта класса
	count = 0
	x = 0
	y = 0
	w = 0
	h = 0
	for det in detection_history[max_history_id]:
		if det is None or det.msg_det.class_id != max_class_id:
			continue
		count += 1
		x += det.msg_det.bbox.x
		y += det.msg_det.bbox.y
		w += det.msg_det.bbox.w
		h += det.msg_det.bbox.h
	x /= count
	y /= count
	w /= count
	h /= count
	print("x", x)
	print("y", y)
	print("w", w)
	print("h", h)
	print("max_class_id", max_class_id)
	print("max_avg_conf", max_avg_conf)
	return [int(x), int(y), int(w), int(h), float(max_avg_conf), int(max_class_id)]


def run_goal_det_pub(goal_det, img_rgb):
	print("===run_goal_det_pub===")
	msg = DetArray()
	msg_det = Det()

	msg_det.bbox.x = goal_det[0]
	msg_det.bbox.y = goal_det[1]
	msg_det.bbox.w = goal_det[2]
	msg_det.bbox.h = goal_det[3]
	msg_det.confidence = goal_det[4]
	msg_det.class_id = goal_det[5]

	msg.dets.append(msg_det)
	msg.img = cv_bridge.cv2_to_imgmsg(img_rgb, "bgr8")
	goal_det_pub.publish(msg)


def det_array_callback(msg):
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
	goal_det = get_goal_det()
	if goal_det is not None:
		run_goal_det_pub(goal_det, img_rgb)


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
