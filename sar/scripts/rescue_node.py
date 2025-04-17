#!/usr/bin/env python3

import rospy
from hum_det.srv import *
from sar.srv import *
import actionlib
from move_base_msgs.msg import MoveBaseAction
import math
import tf.transformations
import tf2_ros
from .utils import get_timeout, load_config, call_action_move_base

# в отчете и тексте по ВКР слово "rescue" заменено на "follow"

NODE_NAME = "rescue_node"

RESCUE_MODE_SWITCH_SRV = "/rescue_mode_switch"
RESCUE_MODE_SWITCH_FEEDBACK_SRV = "/rescue_mode_switch_feedback"

MOVE_BASE_ACTION = "/move_base"

rescue_mode_switch_feedback_client = rospy.ServiceProxy(RESCUE_MODE_SWITCH_FEEDBACK_SRV, ModeSwitch)

move_base_action_client = actionlib.SimpleActionClient(MOVE_BASE_ACTION, MoveBaseAction)

is_on = False
x_pixel = 0
px_pixel = 0
z = 0
f = 0
tf_buffer = tf2_ros.Buffer()
count = 0 # нужен для первого возможного некорректного вызова tf_buffer.lookup_transform
dist_between_bf_lcf = None


# максимальное расстояние от центра системы координат base_footprint до границы footprint
def get_max_to_footprint(footprint):
	return max(math.hypot(x, y) for x, y in footprint)


# расстояние между центрами систем координат base_footprint и левой камеры стереопары. Знак определяется относительно оси X фрейма base_footprint
def get_dist_between_bf_lcf():
	try:
		trans = tf_buffer.lookup_transform(BASE_FRAME, LEFT_CAMERA_FRAME, rospy.Time(0), rospy.Duration(1))
		rospy.loginfo(f"{NODE_NAME}: get_dist_between_bf_lcf: transform found!")
		x, y = trans.transform.translation.x, trans.transform.translation.y
		rospy.loginfo(f"{NODE_NAME}: get_dist_between_bf_lcf: x: {x}")
		rospy.loginfo(f"{NODE_NAME}: get_dist_between_bf_lcf: y: {y}")
		d = math.hypot(x, y)
		if x < 0:
			d *= -1
		return d
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
		rospy.logerr(f"{NODE_NAME}: get_dist_between_bf_lcf: transform not found: %s", str(e))
		return None


def wait_at_victim():
	rospy.loginfo(f"{NODE_NAME}: before at_victim_timeout")
	rospy.sleep(AT_VICTIM_TIMEOUT)
	rospy.loginfo(f"{NODE_NAME}: after at_victim_timeout")


def get_x(x_pixel, px_pixel, z, f): # расчет смещения по оси X изначального фрейма левой камеры стереопары робота
	return (px_pixel - x_pixel) * z / f


def get_alpha(x, z): # расчет угла отклонения объекта от главной оси камеры
	return math.atan(x / z)


def get_camera_pose():
	global count

	while True: # нужен для первого возможного некорректного вызова tf_buffer.lookup_transform
		try:
			trans = tf_buffer.lookup_transform(MAP_FRAME, LEFT_CAMERA_FRAME, rospy.Time(0), rospy.Duration(1))
			rospy.loginfo(f"{NODE_NAME}: transform found!")
			x, y = trans.transform.translation.x, trans.transform.translation.y
			quaternion = trans.transform.rotation
			euler = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
			theta = euler[2]
			return x, y, theta
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
			rospy.logerr(f"{NODE_NAME}: transform not found: %s", str(e))
			if count == 0: # нужен для первого возможного некорректного вызова tf_buffer.lookup_transform
				rospy.loginfo(f"{NODE_NAME}: repeat...")
				count += 1
				continue
			return None


def get_victim_coordinates(x_camera, y_camera, theta, d, alpha):
	x_victim = x_camera + d * math.cos(theta + alpha)
	y_victim = y_camera + d * math.sin(theta + alpha)
	return x_victim, y_victim


def rescue():
	global is_on
	global x_pixel
	global px_pixel
	global z
	global f
	global dist_between_bf_lcf

	if not is_on:
		return
	
	if dist_between_bf_lcf is None:
		dist_between_bf_lcf = get_dist_between_bf_lcf()
	rospy.loginfo(f"{NODE_NAME}: dist_between_bf_lcf: {dist_between_bf_lcf}")
	if dist_between_bf_lcf:
		x = get_x(x_pixel, px_pixel, z, f)
		rospy.loginfo(f"{NODE_NAME}: x: {x}")
		d = math.hypot(x, z)
		rospy.loginfo(f"{NODE_NAME}: d: {d}")
		# if IS_TURTLEBOT3:
		# 	dist_between_bf_lcf = 0.075
		# 	max_to_footprint = 0.257
		# else:
		# 	dist_between_bf_lcf = -0.129
		# 	max_to_footprint = 0.4565
		d = d + dist_between_bf_lcf - MAX_TO_FOOTPRINT - SMALL_DIST_RESERVE
		rospy.loginfo(f"{NODE_NAME}: d + {dist_between_bf_lcf} - {MAX_TO_FOOTPRINT} - {SMALL_DIST_RESERVE}: {d}")
		if d > 0:
			pose = get_camera_pose()
			if pose:
				x_camera, y_camera, theta = pose
				rospy.loginfo(f"{NODE_NAME}: левая камера стереопары робота находится в координатах: x={x_camera}, y={y_camera}, theta={theta}")
				rospy.loginfo(f"{NODE_NAME}: theta in degrees: {math.degrees(theta)}")
				alpha = get_alpha(x, z)
				rospy.loginfo(f"{NODE_NAME}: alpha: {alpha}")
				rospy.loginfo(f"{NODE_NAME}: alpha in degrees: {math.degrees(alpha)}")
				x_victim, y_victim = get_victim_coordinates(x_camera, y_camera, theta, d, alpha)
				rospy.loginfo(f"{NODE_NAME}: координаты пострадавшего: x={x_victim}, y={y_victim}")
				timeout = get_timeout(MAP_FRAME, BASE_FRAME, x_victim, y_victim, tf_buffer, 0.1)
				if timeout:
					timeout *= COEF_TO_VICTIM_TIMEOUT
				else:
					timeout = 10
				rospy.loginfo(f"{NODE_NAME}: result from move_base: {call_action_move_base(MAP_FRAME, x_victim, y_victim, rospy.Duration(timeout), move_base_action_client)}")
				if x_pixel == -1: # если нода det_img_group_node сама выключила данный процесс, то ничего ждать не нужно
					rospy.loginfo(f"{NODE_NAME}: rescue completed!")
					return
				wait_at_victim()
		else:
			wait_at_victim()
	is_on = False
	call_rescue_mode_switch_feedback(is_on)
	rospy.loginfo(f"{NODE_NAME}: rescue completed!")


def handle_rescue_mode_switch(req):
	global is_on
	global x_pixel
	global px_pixel
	global z
	global f

	rospy.loginfo(f"{NODE_NAME}: ===handle_rescue_mode_switch===")
	is_on = req.is_on
	rospy.loginfo(f"{NODE_NAME}: is_on: {is_on}")
	if not is_on:
		x_pixel = -1 # нужно для того, чтобы не уведомлять о выключении ноду det_img_group_node
		move_base_action_client.cancel_goal()
	else:
		x_pixel = req.x_pixel
		px_pixel = req.px_pixel
		z = req.z
		f = req.f
	return RescueModeSwitchResponse(0) # операция прошла успешно


def call_rescue_mode_switch_feedback(msg_is_on):
	rospy.loginfo(f"{NODE_NAME}: ===call_rescue_mode_switch_feedback===")
	try:
		rospy.wait_for_service(RESCUE_MODE_SWITCH_FEEDBACK_SRV, rospy.Duration(5)) # чтобы не было deadlock
		rescue_mode_switch_feedback_client(msg_is_on)
	except (rospy.ServiceException, rospy.ROSException) as e:
		rospy.logerr(f"{NODE_NAME}: error from rescue_mode_switch_feedback: %s" % e)


if __name__ == '__main__':
	try:
		rospy.init_node(NODE_NAME)
		MAP_FRAME = rospy.get_param("map_frame")
		BASE_FRAME = rospy.get_param("base_frame")
		LEFT_CAMERA_FRAME = rospy.get_param("left_camera_frame")
		COEF_TO_VICTIM_TIMEOUT = rospy.get_param(NODE_NAME + "/coef_to_victim_timeout")
		SMALL_DIST_RESERVE = rospy.get_param(NODE_NAME + "/small_dist_reserve")
		AT_VICTIM_TIMEOUT = rospy.get_param(NODE_NAME + "/at_victim_timeout")

		rescue_mode_switch_server = rospy.Service(RESCUE_MODE_SWITCH_SRV, RescueModeSwitch, handle_rescue_mode_switch)

		tf_listener = tf2_ros.TransformListener(tf_buffer)
		config = load_config(rospy.get_param("footprint_path"))
		footprint = config["footprint"]
		rospy.loginfo(f"{NODE_NAME}: footprint: {footprint}")
		MAX_TO_FOOTPRINT = get_max_to_footprint(footprint)
		rospy.loginfo(f"{NODE_NAME}: MAX_TO_FOOTPRINT: {MAX_TO_FOOTPRINT}")

		while True:
			rescue()
			rospy.sleep(1)
	except rospy.ROSInterruptException:
		rospy.loginfo(f"{NODE_NAME}: navigation interrupted")
