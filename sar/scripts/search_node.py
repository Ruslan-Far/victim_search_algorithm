#!/usr/bin/env python3

import rospy
from hum_det.srv import *
import actionlib
from move_base_msgs.msg import MoveBaseAction
import tf2_ros
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from utils import get_timeout, call_action_move_base

# в отчете и тексте по ВКР слово "search" заменено на "snake"

NODE_NAME = "search_node"

SEARCH_MODE_SWITCH_SRV = "/search_mode_switch"

MOVE_BASE_ACTION = "/move_base"

move_base_action_client = actionlib.SimpleActionClient(MOVE_BASE_ACTION, MoveBaseAction)

is_on = False
tf_buffer = tf2_ros.Buffer()


def generate_waypoints(x_min, x_max, y_min, y_max, grid_step):
	waypoints = []
	y = y_min
	direction = 1 # 1 - вправо, -1 - влево

	while y <= y_max:
		if direction == 1:
			x_range = range(int(x_min), int(x_max) + 1, grid_step)
		else:
			x_range = range(int(x_max), int(x_min) - 1, -grid_step)
		for x in x_range:
			waypoints.append((x, y))
		y += grid_step
		direction *= -1 # меняем направление
	return waypoints


def search():
	global is_on
	
	map_frame = rospy.get_param("map_frame")
	base_frame = rospy.get_param("base_frame")
	x_min = rospy.get_param("x_min")
	x_max = rospy.get_param("x_max")
	y_min = rospy.get_param("y_min")
	y_max = rospy.get_param("y_max")
	coef_ptp_timeout = rospy.get_param(NODE_NAME + "/coef_ptp_timeout")
	grid_step = rospy.get_param(NODE_NAME + "/grid_step")
	waypoints = generate_waypoints(x_min, x_max, y_min, y_max, grid_step)
	last_idx = 1

	while last_idx <= len(waypoints):
		if not is_on:
			rospy.sleep(1)
			continue
		if last_idx < len(waypoints):
			x, y = waypoints[last_idx]
			rospy.loginfo(f"-------{NODE_NAME}: moving to: ({x}, {y})")
		else:
			x, y = waypoints[0]
			rospy.loginfo(f"-------{NODE_NAME}: returning to start point: ({x}, {y})")
		timeout = get_timeout(map_frame, base_frame, x, y, tf_buffer, 0.1)
		if timeout:
			timeout *= coef_ptp_timeout
		else:
			timeout = 10
		rospy.loginfo(f"-------{NODE_NAME}: timeout: {timeout}")
		rospy.loginfo(f"-------{NODE_NAME}: result from move_base: {call_action_move_base(map_frame, x, y, rospy.Duration(timeout), move_base_action_client)}")
		if is_on: # нужно для возобновления движения к точке, которое прервали
			last_idx += 1
	rospy.loginfo(f"-------{NODE_NAME}: search completed!")


def handle_search_mode_switch(req):
	global is_on

	rospy.loginfo(f"-------{NODE_NAME}: ===handle_search_mode_switch===")
	is_on = req.is_on
	rospy.loginfo(f"-------{NODE_NAME}: is_on: {is_on}")
	if not is_on:
		move_base_action_client.cancel_goal()
	return ModeSwitchResponse(0) # операция прошла успешно


if __name__ == '__main__':
	try:
		rospy.init_node(NODE_NAME)

		search_mode_switch_server = rospy.Service(SEARCH_MODE_SWITCH_SRV, ModeSwitch, handle_search_mode_switch)

		tf_listener = tf2_ros.TransformListener(tf_buffer)
		search()
	except rospy.ROSInterruptException:
		rospy.loginfo(f"-------{NODE_NAME}: navigation interrupted")
