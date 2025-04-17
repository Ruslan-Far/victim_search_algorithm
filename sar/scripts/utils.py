import yaml
import rospy
import math
import tf2_ros
from move_base_msgs.msg import MoveBaseGoal


def get_timeout(frame, frame2, x_goal, y_goal, tf_buffer, speed):
	try:
		trans = tf_buffer.lookup_transform(frame, frame2, rospy.Time(0), rospy.Duration(1))
		rospy.loginfo("utils: transform found!")
		x, y = trans.transform.translation.x, trans.transform.translation.y
		rospy.loginfo(f"utils: x: {x}")
		rospy.loginfo(f"utils: y: {y}")
		d = math.hypot(x - x_goal, y - y_goal)
		rospy.loginfo(f"utils: d: {d}")
		return d / speed
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
		rospy.logerr("utils: transform not found: %s", str(e))
		return None


def load_config(path):
    file = open(path, "r", encoding="utf-8")
    config = yaml.safe_load(file)
    file.close()
    return config


def call_action_move_base(frame_id, x, y, timeout, move_base_action_client):
	rospy.loginfo("utils: ===call_action_move_base===")
	move_base_action_client.wait_for_server()
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = frame_id
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y = y
	goal.target_pose.pose.orientation.w = 1.0
	move_base_action_client.send_goal(goal)
	move_base_action_client.wait_for_result(timeout)
	return move_base_action_client.get_result()
