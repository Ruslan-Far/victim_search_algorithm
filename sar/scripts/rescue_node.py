#!/usr/bin/env python3

import rospy
from hum_det.srv import *
from sar.srv import *
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math
# import tf
# import tf.transformations
import tf2_ros

NODE_NAME = "rescue_node"

RESCUE_MODE_SWITCH_SRV = "rescue_mode_switch"
RESCUE_MODE_SWITCH_FEEDBACK_SRV = "rescue_mode_switch_feedback"

MOVE_BASE_ACTION = "move_base"

rescue_mode_switch_feedback_client = rospy.ServiceProxy(RESCUE_MODE_SWITCH_FEEDBACK_SRV, DetModeSwitch)

move_base_action_client = actionlib.SimpleActionClient(MOVE_BASE_ACTION, MoveBaseAction)

is_on = False
x = 0
cx = 0
distance = 0
f = 0


def get_victim_coordinates(robot_x, robot_y, theta, distance, alpha):
	victim_x = robot_x + distance * math.cos(theta + alpha)
	victim_y = robot_y + distance * math.sin(theta + alpha)
	return victim_x, victim_y


def get_robot_pose():
	# listener = tf.TransformListener()
	# try:
	# 	listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(5))
	# 	(trans, rot) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
	# 	x, y = trans[0], trans[1]  # Позиция робота
	# 	quaternion = rot           # Кватернион
	# 	euler = tf.transformations.euler_from_quaternion(quaternion)
	# 	theta = euler[2]  # Угол поворота (в радианах)
	# 	return x, y, theta
	# except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	# 	rospy.logerr("ошибка при получении координат робота")
	# 	return None
	tf_buffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tf_buffer)
	# Ожидаем трансформацию, если она не существует
	try:
		# tf_buffer.wait_for_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
		transform = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1))
		rospy.loginfo("Transform found!")
		rospy.loginfo(f"{transform[0]}, {transform[1]}")
		return None
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
		rospy.logerr("Transform not found: %s", str(e))
		return None


def get_alpha(x, cx, distance, f):
	horizontal_offset_x = (cx - x) * distance / f
	alpha = math.atan(horizontal_offset_x / distance)
	rospy.loginfo(f"alpha: {alpha}")
	return math.radians(alpha)


def rescue():
	global is_on
	global x
	global cx
	global distance
	global f

	pose = get_robot_pose()
	if pose:
		rospy.loginfo(f"робот находится в координатах: x={pose[0]}, y={pose[1]}, theta={pose[2]}")
	if not is_on:
		return

	pose = get_robot_pose()
	if pose:
		robot_x, robot_y, theta = pose # получаем текущие координаты робота
		alpha = get_alpha(x, cx, distance, f)
		rospy.loginfo(f"alpha in radians: {alpha}")
		victim_x, victim_y = get_victim_coordinates(robot_x, robot_y, theta, distance, alpha)
		rospy.loginfo(f"координаты пострадавшего: x={victim_x}, y={victim_y}")
		timeout = rospy.Duration(180)
		rospy.loginfo(f"result from move_base: {call_action_move_base(victim_x, victim_y, timeout)}")
		if x == -1: # если нода det_img_group_node сама выключила данный процесс, то ничего ждать не нужно
			rospy.loginfo("rescue completed!")
			return
		rospy.sleep(10) # время для спасения пострадавшего
	is_on = False
	call_rescue_mode_switch_feedback(is_on)
	rospy.loginfo("rescue completed!")


def handle_rescue_mode_switch(req):
	global is_on
	global x
	global cx
	global distance
	global f

	rospy.loginfo("===handle_rescue_mode_switch===")
	is_on = req.is_on
	rospy.loginfo(f"is_on: {is_on}")
	if not is_on:
		x = -1 # нужно для того, чтобы не уведомлять о выключении ноду det_img_group_node
		move_base_action_client.cancel_goal()
	else:
		x = req.x
		cx = req.cx
		distance = req.distance
		f = req.f
	return RescueModeSwitchResponse(0) # операция прошла успешно


def call_rescue_mode_switch_feedback(msg_is_on):
	rospy.loginfo("===call_rescue_mode_switch_feedback===")
	try:
		rospy.wait_for_service(RESCUE_MODE_SWITCH_FEEDBACK_SRV, rospy.Duration(5)) # чтобы не было deadlock
		rescue_mode_switch_feedback_client(msg_is_on)
	except (rospy.ServiceException, rospy.ROSException) as e:
		rospy.logerr("error from rescue_mode_switch_feedback: %s" % e)


def call_action_move_base(x, y, timeout):
	rospy.loginfo("===call_action_move_base===")
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

		rescue_mode_switch_server = rospy.Service(RESCUE_MODE_SWITCH_SRV, RescueModeSwitch, handle_rescue_mode_switch)

		while True:
			rescue()
			rospy.sleep(1)
	except rospy.ROSInterruptException:
		rospy.loginfo("navigation interrupted")
