import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point


def move_to_goal(goal_x, goal_y):
    # define a client for to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("map_server", MoveBaseAction)
    # wait for the action server to come up
    while not ac.wait_for_server(rospy.Duration.from_sec(5.0)):
        rospy.loginfo("Waiting for the map_server action server to come up")
    goal = MoveBaseGoal()
    # "map" considers the given goal coordinates to be global while "base_link" would consider them to be local from
    # the roboter
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal
    goal.target_pose.pose.position = Point(goal_x, goal_y, 0)
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0
    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)
    # blocking request, waits until roboter either reaches goal or gets interrupted
    # maybe dont use this line to interrupt current request?
    ac.wait_for_result(rospy.Duration(60))
    if ac.get_state() == GoalStatus.SUCCEEDED:
        rospy.loginfo("You have reached the destination")
        return True
    else:
        rospy.loginfo("The robot failed to reach the destination")
        return False


if __name__ == '__main__':
    rospy.init_node('slam_navigation')
    move_to_goal(3, 0)
    rospy.spin()
