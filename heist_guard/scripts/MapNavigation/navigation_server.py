import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction


class MoveToGoalAction(object):
    # create messages that are used to publish feedback/result

    def __init__(self, name):
        self.feedback = ''
        self.result = ''
        # rospy.Subscriber("distance_to_goal", self.distance_callback)
        self.distance = 0
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, MoveBaseAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def distance_callback(self, data):
        self.distance = data

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        self._as.publish_feedback()


if __name__ == '__main__':
    rospy.init_node('nav_server')
    server = MoveToGoalAction(rospy.get_name())
    rospy.spin()
