#!/usr/bin/env python3

import rospy
from hum_det.srv import *
from sar.srv import *
import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction
import math
import tf.transformations
import tf2_ros
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from utils import load_config, call_action_move_base

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
is_person_class = False
tf_buffer = tf2_ros.Buffer()
count = 0 # нужен для первого возможного некорректного вызова tf_buffer.lookup_transform


# максимальное расстояние от центра системы координат base_footprint до границы footprint
def get_max_to_footprint(footprint):
	return max(math.hypot(x, y) for x, y in footprint)


def rotate_at_victim():
	rospy.loginfo(f"{NODE_NAME}: ===rotate_at_victim===")
	move_base_action_client.cancel_goal()
	run_cmd_vel_pub(0)
	run_cmd_vel_pub(AT_VICTIM_ANGULAR_SPEED)
	rospy.sleep(AT_VICTIM_ROTATION_TIMEOUT)
	run_cmd_vel_pub(0)
	run_cmd_vel_pub(-AT_VICTIM_ANGULAR_SPEED)
	rospy.sleep(2 * AT_VICTIM_ROTATION_TIMEOUT)
	run_cmd_vel_pub(0)
	run_cmd_vel_pub(AT_VICTIM_ANGULAR_SPEED)
	rospy.sleep(AT_VICTIM_ROTATION_TIMEOUT)
	run_cmd_vel_pub(0)


def wait_at_victim():
	rospy.loginfo(f"{NODE_NAME}: before at_victim_timeout")
	rospy.sleep(AT_VICTIM_TIMEOUT)
	rospy.loginfo(f"{NODE_NAME}: after at_victim_timeout")


def get_x(x_pixel, px_pixel, z, f): # расчет смещения по оси X изначального фрейма левой камеры стереопары робота
	return (px_pixel - x_pixel) * z / f


def get_alpha(x, z): # расчет угла отклонения объекта от главной оси камеры
	return math.atan(x / z)


def get_pose(frame, frame2):
	global count

	while True: # нужен для первого возможного некорректного вызова tf_buffer.lookup_transform
		try:
			trans = tf_buffer.lookup_transform(frame, frame2, rospy.Time(0), rospy.Duration(1))
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


def get_goal_coordinates_and_timeout(x_base, y_base, x_victim, y_victim, safe_radius, speed):
	dx = x_victim - x_base
	dy = y_victim - y_base
	d = math.hypot(dx, dy)
	rospy.loginfo(f"{NODE_NAME}: get_goal_coordinates_and_timeout: d: {d}")
	d_safe = d - safe_radius
	rospy.loginfo(f"{NODE_NAME}: get_goal_coordinates_and_timeout: d_safe: {d_safe}")
	if d_safe <= 0:
		rospy.loginfo(f"{NODE_NAME}: get_goal_coordinates_and_timeout: робот уже находится слишком близко к пострадавшему!")
		return None
	direction_x = dx / d
	direction_y = dy / d
	x_goal = x_base + d_safe * direction_x
	y_goal = y_base + d_safe * direction_y
	timeout = d_safe / speed
	return x_goal, y_goal, timeout


def rescue():
	global is_on
	global x_pixel
	global px_pixel
	global z
	global f
	global is_person_class

	if not is_on:
		return

	camera_pose = get_pose(MAP_FRAME, LEFT_CAMERA_FRAME)
	base_pose = get_pose(MAP_FRAME, BASE_FRAME)
	if camera_pose and base_pose:
		x_camera, y_camera, theta = camera_pose
		rospy.loginfo(f"{NODE_NAME}: левая камера стереопары робота находится в координатах: x={x_camera}, y={y_camera}, theta={theta}")
		rospy.loginfo(f"{NODE_NAME}: theta in degrees: {math.degrees(theta)}")
		x = get_x(x_pixel, px_pixel, z, f)
		rospy.loginfo(f"{NODE_NAME}: x: {x}")
		d = math.hypot(x, z)
		rospy.loginfo(f"{NODE_NAME}: d: {d}")
		alpha = get_alpha(x, z)
		rospy.loginfo(f"{NODE_NAME}: alpha: {alpha}")
		rospy.loginfo(f"{NODE_NAME}: alpha in degrees: {math.degrees(alpha)}")
		x_victim, y_victim = get_victim_coordinates(x_camera, y_camera, theta, d, alpha)
		rospy.loginfo(f"{NODE_NAME}: координаты пострадавшего: x={x_victim}, y={y_victim}")
		x_base, y_base, _ = base_pose
		rospy.loginfo(f"{NODE_NAME}: база робота находится в координатах: x={x_base}, y={y_base}")
		safe_radius = MAX_TO_FOOTPRINT
		if is_person_class:
			safe_radius += PERSON_SMALL_DIST_RESERVE
		else:
			safe_radius += SMALL_DIST_RESERVE
		goal_coordinates_and_timeout = get_goal_coordinates_and_timeout(x_base, y_base, x_victim, y_victim, safe_radius, AVG_VEL_X)
		if goal_coordinates_and_timeout:
			x_goal, y_goal, timeout = goal_coordinates_and_timeout
			rospy.loginfo(f"{NODE_NAME}: целевые координаты: x={x_goal}, y={y_goal}")
			timeout *= COEF_TO_VICTIM_TIMEOUT
			rospy.loginfo(f"{NODE_NAME}: timeout: {timeout}")
			rospy.loginfo(f"{NODE_NAME}: result from move_base: {call_action_move_base(MAP_FRAME, x_goal, y_goal, rospy.Duration(timeout), move_base_action_client)}")
			if x_pixel == -1: # если нода det_img_group_node сама выключила данный процесс, то ничего ждать не нужно
				rospy.loginfo(f"{NODE_NAME}: rescue completed!")
				return
			rotate_at_victim()
			wait_at_victim()
		else:
			rotate_at_victim()
			wait_at_victim()
	is_on = False
	call_rescue_mode_switch_feedback(is_on)
	rospy.loginfo(f"{NODE_NAME}: rescue completed!")


def run_cmd_vel_pub(angular_speed):
	msg = Twist()

	msg.linear.x = 0
	msg.linear.y = 0
	msg.linear.z = 0
	msg.angular.x = 0
	msg.angular.y = 0
	msg.angular.z = angular_speed
	cmd_vel_pub.publish(msg)
	rospy.loginfo(f"{NODE_NAME}: angular_speed: {angular_speed}")


def handle_rescue_mode_switch(req):
	global is_on
	global x_pixel
	global px_pixel
	global z
	global f
	global is_person_class

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
		is_person_class = req.is_person_class
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
		PERSON_SMALL_DIST_RESERVE = rospy.get_param(NODE_NAME + "/person_small_dist_reserve")
		AT_VICTIM_ANGULAR_SPEED = rospy.get_param(NODE_NAME + "/at_victim_angular_speed")
		AT_VICTIM_ROTATION_TIMEOUT = rospy.get_param(NODE_NAME + "/at_victim_rotation_timeout")
		AT_VICTIM_TIMEOUT = rospy.get_param(NODE_NAME + "/at_victim_timeout")

		cmd_vel_pub = rospy.Publisher(rospy.get_param("cmd_vel_topic"), Twist, queue_size=1)

		rescue_mode_switch_server = rospy.Service(RESCUE_MODE_SWITCH_SRV, RescueModeSwitch, handle_rescue_mode_switch)

		tf_listener = tf2_ros.TransformListener(tf_buffer)
		config = load_config(rospy.get_param("footprint_path"))
		footprint = config["footprint"]
		rospy.loginfo(f"{NODE_NAME}: footprint: {footprint}")
		MAX_TO_FOOTPRINT = get_max_to_footprint(footprint)
		rospy.loginfo(f"{NODE_NAME}: MAX_TO_FOOTPRINT: {MAX_TO_FOOTPRINT}")
		config = load_config(rospy.get_param("max_vel_x_path"))
		AVG_VEL_X = config["DWAPlannerROS"]["max_vel_x"] / 2
		rospy.loginfo(f"{NODE_NAME}: AVG_VEL_X: {AVG_VEL_X}")
		while True:
			rescue()
			rospy.sleep(1)
	except rospy.ROSInterruptException:
		rospy.loginfo(f"{NODE_NAME}: navigation interrupted")
