#!/usr/bin/env python3

import rospy
from hum_det.srv import *
from sar.srv import *
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math
import tf
import tf.transformations
import tf2_ros

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


def get_x(x_pixel, px_pixel, z, f): # расчет смещения по оси X изначального фрейма левой камеры стереопары робота
	return (px_pixel - x_pixel) * z / f


def get_d(x, z): # расчет Евклидова расстояния до объекта
	return math.sqrt(x ** 2 + z ** 2)


def get_alpha(x, z): # расчет угла отклонения объекта от главной оси камеры
	return math.atan(x / z)


def get_camera_pose():
	global count

	while True: # нужен для первого возможного некорректного вызова tf_buffer.lookup_transform
		try:
			if IS_TURTLEBOT3:
				trans = tf_buffer.lookup_transform("map", "wide_stereo_l_stereo_camera_frame", rospy.Time(0), rospy.Duration(1)) # turtlebot3
			else:
				trans = tf_buffer.lookup_transform("engineer/map", "engineer/camera3_link", rospy.Time(0), rospy.Duration(1)) # engineer
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

	if not is_on:
		return

	x = get_x(x_pixel, px_pixel, z, f)
	rospy.loginfo(f"{NODE_NAME}: x: {x}")
	d = get_d(x, z)
	rospy.loginfo(f"{NODE_NAME}: d: {d}")
	# максимальное расстояние от центра системы координат base_footprint до границы footprint {
	if IS_TURTLEBOT3:
		max_to_footprint = 0.257 # m (turtlebot3)
	else:
		max_to_footprint = 0.4565 # m (engineer)
	# }
	d = d - max_to_footprint - SMALL_DIST_RESERVE
	rospy.loginfo(f"{NODE_NAME}: d - {max_to_footprint} - {SMALL_DIST_RESERVE}: {d}")
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
			timeout = rospy.Duration(TO_VICTIM_TIMEOUT)
			rospy.loginfo(f"{NODE_NAME}: result from move_base: {call_action_move_base(x_victim, y_victim, timeout)}")
			if x_pixel == -1: # если нода det_img_group_node сама выключила данный процесс, то ничего ждать не нужно
				rospy.loginfo(f"{NODE_NAME}: rescue completed!")
				return
			rospy.loginfo(f"{NODE_NAME}: before at_victim_timeout")
			rospy.sleep(AT_VICTIM_TIMEOUT)
			rospy.loginfo(f"{NODE_NAME}: after at_victim_timeout")
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


def call_action_move_base(x, y, timeout):
	rospy.loginfo(f"{NODE_NAME}: ===call_action_move_base===")
	move_base_action_client.wait_for_server()
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y = y
	goal.target_pose.pose.orientation.w = 1.0
	move_base_action_client.send_goal(goal)
	move_base_action_client.wait_for_result(timeout)
	return move_base_action_client.get_result()


if __name__ == '__main__':
	try:
		rospy.init_node(NODE_NAME)
		IS_TURTLEBOT3 = rospy.get_param(NODE_NAME + "/is_turtlebot3")
		SMALL_DIST_RESERVE = rospy.get_param(NODE_NAME + "/small_dist_reserve")
		TO_VICTIM_TIMEOUT = rospy.get_param(NODE_NAME + "/to_victim_timeout")
		AT_VICTIM_TIMEOUT = rospy.get_param(NODE_NAME + "/at_victim_timeout")

		rescue_mode_switch_server = rospy.Service(RESCUE_MODE_SWITCH_SRV, RescueModeSwitch, handle_rescue_mode_switch)

		tf_listener = tf2_ros.TransformListener(tf_buffer)
		while True:
			rescue()
			rospy.sleep(1)
	except rospy.ROSInterruptException:
		rospy.loginfo(f"{NODE_NAME}: navigation interrupted")
