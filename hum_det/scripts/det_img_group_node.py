#!/usr/bin/env python3

import rospy
from hum_det.msg import DetArray, Det, Box
from sensor_msgs.msg import Image
from hum_det.srv import *
from sar.srv import *
from cv_bridge import CvBridge
import time
import numpy as np
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from det_class import DetClass

IS_TURTLEBOT3 = True # если запуск алгоритма будет производиться на роботе Turtlebot3. Для робота Инженер необходимо поставить False

NODE_NAME = "det_img_group_node"

DET_ARRAY_TOPIC = "/det_array"
GOAL_DET_TOPIC = "/goal_det"
NORM_DEPTH_MAP_TOPIC = "/depth_map"

DET_GROUP_MODE_SWITCH_SRV = "/det_group_mode_switch"
STEREO_MODE_SRV = "/stereo_mode"
SEARCH_MODE_SWITCH_SRV = "/search_mode_switch"
RESCUE_MODE_SWITCH_SRV = "/rescue_mode_switch"
RESCUE_MODE_SWITCH_FEEDBACK_SRV = "/rescue_mode_switch_feedback"

MAX_FRAMES = 4
MIN_DETECTION_RATE = 0.7
MIN_IOU = 0.8
MAX_STOP_TIME = 20 # seconds
MIN_MOVING_TIME = 5 # seconds
# максимальное расстояние от центра системы координат base_footprint до границы footprint {
if IS_TURTLEBOT3:
	MAX_TO_FOOTPRINT = 0.205 # m (turtlebot3)
else:
	MAX_TO_FOOTPRINT = 0.3 # m (engineer)
# }
MIN_DISTANCE = MAX_TO_FOOTPRINT + 0.01 # m (прибавлено расстояние в качестве небольшого запаса)

goal_det_pub = rospy.Publisher(GOAL_DET_TOPIC, DetArray, queue_size=1)
norm_depth_map_pub = rospy.Publisher(NORM_DEPTH_MAP_TOPIC, Image, queue_size=1)

stereo_mode_client = rospy.ServiceProxy(STEREO_MODE_SRV, StereoMode)
search_mode_switch_client = rospy.ServiceProxy(SEARCH_MODE_SWITCH_SRV, ModeSwitch)
rescue_mode_switch_client = rospy.ServiceProxy(RESCUE_MODE_SWITCH_SRV, RescueModeSwitch)

cv_bridge = CvBridge()
is_on = False
is_on_search = False
is_on_rescue = False
detection_history = []
is_min_moving = False # нужно, чтобы старые изображения (которые в пределах MIN_MOVING_TIME) не обрабатывались вообще уже во время работы search_mode
start_time = 0


def reset_fields():
	global detection_history
	global is_min_moving
	global start_time

	detection_history = []
	is_min_moving = False
	start_time = 0
	rospy.loginfo("reseted_fields")


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

	# rospy.loginfo("===fit_detection_history===")
	old_len_detection_history = len(detection_history) # чтобы не делать лишние последние итерации в цикле
	# rospy.loginfo(f"old_len_detection_history: {old_len_detection_history}")
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

	# rospy.loginfo("===get_goal_det===")
	max_history_id = -1
	max_detection_rate = MIN_DETECTION_RATE
	max_class_id = -1
	max_avg_conf = -1
	for hist_id, history in enumerate(detection_history):
		# если данные еще не накопились в полном объеме
		if len(history) != MAX_FRAMES:
			# rospy.loginfo("данные еще не накопились в полном объеме")
			continue
		# rospy.loginfo("----------------------------------------------------------------- step 1")
		tmp_rates = [0, 0, 0, 0] # foot, head, hand, person
		for det in history:
			if det is None:
				continue
			tmp_rates[det.msg_det.class_id] += 1
		for cls_id in range(len(tmp_rates)):
			tmp_rates[cls_id] /= len(history)
		tmp_rate = max(tmp_rates)
		if tmp_rate < max_detection_rate:
			# rospy.loginfo("коэффициент обнаружения объекта класса оказался меньше необходимого порога {")
			# rospy.loginfo(f"tmp_rate: {tmp_rate}")
			# rospy.loginfo(f"tmp_class_id: {tmp_rates.index(tmp_rate)}")
			# rospy.loginfo("}")
			continue
		# rospy.loginfo("----------------------------------------------------------------- step 2")
		tmp_class_id = tmp_rates.index(tmp_rate)
		count = 0
		tmp_avg_conf = 0
		for det in history:
			if det is None or det.msg_det.class_id != tmp_class_id:
				continue
			count += 1
			tmp_avg_conf += det.msg_det.confidence
		tmp_avg_conf /= count
		# rospy.loginfo(f"tmp_class_id: {tmp_class_id}")
		# rospy.loginfo(f"count: {count}")
		# rospy.loginfo(f"tmp_avg_conf: {tmp_avg_conf}")
		if tmp_rate == max_detection_rate:
			# rospy.loginfo("tmp_rate == max_detection_rate")
			if tmp_avg_conf <= max_avg_conf:
				# rospy.loginfo("tmp_avg_conf <= max_avg_conf")
				continue
		max_history_id = hist_id
		max_detection_rate = tmp_rate
		max_class_id = tmp_class_id
		max_avg_conf = tmp_avg_conf
		# rospy.loginfo(f"max_history_id: {max_history_id}")
		# rospy.loginfo(f"max_detection_rate: {max_detection_rate}")
		# rospy.loginfo(f"max_class_id: {max_class_id}")
		# rospy.loginfo(f"max_avg_conf: {max_avg_conf}")
	# если не накопилось достаточно данных или все коэффициенты обнаружений объектов классов оказались меньше необходимого порога
	if max_class_id == -1:
		# rospy.loginfo("не накопилось достаточно данных или все коэффициенты обнаружений объектов классов оказались меньше необходимого порога")
		return None
	rospy.loginfo("----------------------------------------------------------------- step 3")
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
	rospy.loginfo(f"x: {x}")
	rospy.loginfo(f"y: {y}")
	rospy.loginfo(f"w: {w}")
	rospy.loginfo(f"h: {h}")
	rospy.loginfo(f"max_class_id: {max_class_id}")
	rospy.loginfo(f"max_avg_conf: {max_avg_conf}")
	if x < 0:
		x = 0
		rospy.loginfo(f"corrected x: {x}")
	if y < 0:
		y = 0
		rospy.loginfo(f"corrected y: {y}")
	return [int(x), int(y), int(w), int(h), float(max_avg_conf), int(max_class_id)]


def start_stereo_rescue_modes(goal_det, msg_img, msg_disp_img):
	global is_on_search
	global is_on_rescue
	global start_time

	rospy.loginfo("===start_stereo_rescue_modes===")
	msg_box = Box()

	msg_box.x = goal_det[0]
	msg_box.y = goal_det[1]
	msg_box.w = goal_det[2]
	msg_box.h = goal_det[3]
	if msg_disp_img.f != -1:
		distance = call_stereo_mode(msg_box, msg_disp_img)
	else:
		distance = -1
	run_goal_det_pub(msg_box, goal_det[4], goal_det[5], msg_img, distance)
	rospy.loginfo(f"distance: {distance}")
	rospy.loginfo(f"distance - MIN_DISTANCE: {distance - MIN_DISTANCE}")
	if distance != -1 and distance != 0 and distance - MIN_DISTANCE > 0:
		rospy.loginfo("включить rescue_mode через сервис")
		call_rescue_mode_switch(True, int(msg_box.x + msg_box.w / 2), int(msg_disp_img.image.width / 2), distance - MIN_DISTANCE, msg_disp_img.f)
		if is_on_rescue:
			start_time = time.time() # нужно для подстраховки на случай того, если rescue_mode не уведомит о завершении своей работы по каким-либо техническим причинам
			return
	reset_fields()
	rospy.loginfo("4444444444444444444 включить search_mode через сервис")
	call_search_mode_switch(True)


def run_goal_det_pub(msg_box, confidence, class_id, msg_img, distance):
	rospy.loginfo("===run_goal_det_pub===")
	msg = DetArray()
	msg_det = Det()

	msg_det.bbox = msg_box
	msg_det.confidence = confidence
	msg_det.class_id = class_id

	msg.dets.append(msg_det)
	msg.img = msg_img
	msg.distance = distance
	goal_det_pub.publish(msg)


def run_norm_depth_map_pub(msg):
	disp_img_image = cv_bridge.imgmsg_to_cv2(msg.disp_img.image, desired_encoding="32FC1")
	depth_map = np.where(disp_img_image >= msg.disp_img.min_disparity, msg.disp_img.f * msg.disp_img.T / disp_img_image, 0)
	max_distance = msg.disp_img.f * msg.disp_img.T / msg.disp_img.min_disparity # msg.disp_img.min_disparity должен быть >= 1
	norm_depth_map = (255 * depth_map / max_distance).astype(np.uint8)
	norm_depth_map_pub.publish(cv_bridge.cv2_to_imgmsg(norm_depth_map, "mono8"))


def det_array_callback(msg):
	global is_on
	global is_on_search
	global is_on_rescue
	global detection_history
	global is_min_moving
	global start_time

	if not is_on:
		return
	if is_on_rescue:
		if time.time() - start_time >= 1000: # нужно для подстраховки на случай того, если rescue_mode не уведомит о завершении своей работы по каким-либо техническим причинам
			rospy.loginfo("2222 выключить rescue_mode через сервис")
			call_rescue_mode_switch(False, 0, 0, 0, 0)
			reset_fields()
			rospy.loginfo("after 1000 seconds")
			rospy.loginfo("3333333333333333333 включить search_mode через сервис")
			call_search_mode_switch(True)
		else:
			return
	elif is_min_moving:
		if time.time() - start_time >= MIN_MOVING_TIME:
			reset_fields()
			rospy.loginfo("after MIN_MOVING_TIME seconds")
		else:
			return

	# если будут наблюдаться проблемы с производительностью, то можно закомментировать
	if msg.disp_img.f != -1:
		run_norm_depth_map_pub(msg)

	if not is_on_search:
		if time.time() - start_time >= MAX_STOP_TIME:
			rospy.loginfo("222222222222222 включить search_mode через сервис")
			call_search_mode_switch(True)
			if is_on_search: # нужно для того, чтобы иметь возможность подъехать к пострадавшему даже при отказе работы search_mode
				is_min_moving = True
				start_time = time.time()
				return
	elif len(msg.dets) != 0: # нужно как можно быстрее остановиться, когда робот кого-то увидел
		rospy.loginfo("выключить search_mode через сервис")
		call_search_mode_switch(False)
		start_time = time.time()
	fit_detection_history(msg.dets)
	goal_det = get_goal_det()
	if goal_det is not None:
		start_stereo_rescue_modes(goal_det, msg.img, msg.disp_img)


# переключаем режим Autonomous Victim Search во вкл/выкл состояние
def handle_det_group_mode_switch(req):
	global is_on
	global is_on_search
	global is_on_rescue

	is_on = req.is_on
	if is_on: # первым делом при включении алгоритма обязательно нужно сбросить все переменные
		reset_fields()
		rospy.loginfo("включить search_mode через сервис")
		call_search_mode_switch(True)
	else:
		rospy.loginfo("2222222222222222 выключить search_mode через сервис")
		call_search_mode_switch(False)
		rospy.loginfo("выключить rescue_mode через сервис")
		call_rescue_mode_switch(False, 0, 0, 0, 0)
	return ModeSwitchResponse(0) # операция прошла успешно


def handle_rescue_mode_switch_feedback(req):
	global is_on_rescue

	rospy.loginfo("===handle_rescue_mode_switch_feedback===")
	is_on_rescue = req.is_on
	reset_fields()
	rospy.loginfo("555555555555555555555 включить search_mode через сервис")
	call_search_mode_switch(True)
	return ModeSwitchResponse(0) # операция прошла успешно


def call_stereo_mode(msg_box, msg_disp_img):
	rospy.loginfo("===call_stereo_mode===")
	try:
		res = stereo_mode_client(msg_box, msg_disp_img)
		return res.distance
	except rospy.ServiceException as e:
		rospy.logerr("error from stereo_mode: %s" % e)
		return -1


def call_search_mode_switch(msg_is_on):
	global is_on_search

	rospy.loginfo("===call_search_mode_switch===")
	try:
		search_mode_switch_client(msg_is_on)
		if msg_is_on:
			rospy.loginfo("++++++++++++++++++++++++++++++++++++++++RUN++++++++++++++++++++++++++++++++++++++++")
		else:
			rospy.loginfo("----------------------------------------STOP----------------------------------------")
		is_on_search = msg_is_on
	except rospy.ServiceException as e:
		rospy.logerr("error from search_mode_switch: %s" % e)
		is_on_search = False


def call_rescue_mode_switch(msg_is_on, msg_x, msg_cx, msg_distance, msg_f):
	global is_on_rescue

	rospy.loginfo("===call_rescue_mode_switch===")
	try:
		rescue_mode_switch_client(msg_is_on, msg_x, msg_cx, msg_distance, msg_f)
		is_on_rescue = msg_is_on
	except rospy.ServiceException as e:
		rospy.logerr("error from rescue_mode_switch: %s" % e)
		is_on_rescue = False


def main() -> None:
	rospy.init_node(NODE_NAME)

	det_array_sub = rospy.Subscriber(DET_ARRAY_TOPIC, DetArray, det_array_callback, queue_size=1)

	det_group_mode_switch_server = rospy.Service(DET_GROUP_MODE_SWITCH_SRV, ModeSwitch, handle_det_group_mode_switch)
	rescue_mode_switch_feedback_server = rospy.Service(RESCUE_MODE_SWITCH_FEEDBACK_SRV, ModeSwitch, handle_rescue_mode_switch_feedback)

	rospy.spin()


if __name__ == '__main__':
	main()
