#!/usr/bin/env python3

import rospy
from hum_det.srv import *
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

NODE_NAME = "search_node"

SEARCH_MODE_SWITCH_SRV = "search_mode_switch"

MOVE_BASE_ACTION = "move_base"

move_base_action_client = actionlib.SimpleActionClient(MOVE_BASE_ACTION, MoveBaseAction)

is_on = False


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
	
	x_min = rospy.get_param("x_min")
	x_max = rospy.get_param("x_max")
	y_min = rospy.get_param("y_min")
	y_max = rospy.get_param("y_max")
	grid_step = rospy.get_param("grid_step") # шаг сетки
	waypoints = generate_waypoints(x_min, x_max, y_min, y_max, grid_step)
	last_idx = 1

	while last_idx <= len(waypoints):
		if not is_on:
			rospy.loginfo("search stopped due to victim detection!")
			rospy.sleep(1)
			continue
		if last_idx < len(waypoints):
			x, y = waypoints[last_idx]
			timeout = rospy.Duration(20)
			rospy.loginfo(f"moving to: ({x}, {y})")
		else:
			x, y = waypoints[0]
			timeout = rospy.Duration(180)
			rospy.loginfo(f"returning to start point: ({x}, {y})")
		rospy.loginfo(f"result from move_base: {call_action_move_base(x, y, timeout)}")
		if is_on: # нужно для возобновления движения к точке, которое прервали
			last_idx += 1
	rospy.loginfo("search completed!")


def handle_search_mode_switch(req):
	global is_on

	rospy.loginfo("===handle_search_mode_switch===")
	is_on = req.is_on
	rospy.loginfo(f"is_on: {is_on}")
	if not is_on:
		move_base_action_client.cancel_goal()
	return DetModeSwitchResponse(0) # операция прошла успешно


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

		search_mode_switch_server = rospy.Service(SEARCH_MODE_SWITCH_SRV, DetModeSwitch, handle_search_mode_switch)

		search()
	except rospy.ROSInterruptException:
		rospy.loginfo("navigation interrupted")
