#!/usr/bin/env python3.8
import math
import numpy as np
import rospy
from nav_msgs.msg import Odometry

ROBOTER_SPEED = 0.22
TIME_INTERVAL = 2
MAX_TRAVEL_DISTANCE_BETWEEN_INTERVAL = ROBOTER_SPEED * TIME_INTERVAL
MAX_NOISE = 1


def calculate_distance_between_2_points(x1, x2, y1, y2):
    return math.sqrt(math.pow(abs(x1 - x2), 2) + math.pow(abs(y1 - y2), 2))


def get_slope_between_to_points(x1, x2, y1, y2):
    return (y1 - y2) / (x1 - x2)


def get_gradient_angle(slope):
    return math.atan(slope)


# x1 ist hintere Punkte in der Liste, also der i-te Punkt, x2 ist der i-1-te Punkt
def get_points_closer_together(x1, x2, y1, y2):
    distance = calculate_distance_between_2_points(x1, x2, y1, y2)
    while distance > MAX_TRAVEL_DISTANCE_BETWEEN_INTERVAL:
        half_extra_distance = (distance - MAX_TRAVEL_DISTANCE_BETWEEN_INTERVAL) / 2
        slope = abs(get_slope_between_to_points(x1, x2, y1, y2))
        gradient_angle = get_gradient_angle(slope)
        y_extra = abs(half_extra_distance * math.sin(gradient_angle)) + 0.005  # Ansonsten gibt es eine Dauerschleife,
        # da Lim von Distance gegen Max_Travel geht
        x_extra = abs(half_extra_distance * math.cos(gradient_angle)) + 0.005  # Ansonsten gibt es eine Dauerschleife,
        # da Lim von Distance gegen Max_Travel geht
        if x1 > x2:
            x1 = x1 - x_extra
            x2 = x2 + x_extra
        else:
            x1 = x1 + x_extra
            x2 = x2 - x_extra

        if y1 > y2:
            y1 = y1 - y_extra
            y2 = y2 + y_extra
        else:
            y1 = y1 + y_extra
            y2 = y2 - y_extra

        distance = calculate_distance_between_2_points(x1, x2, y1, y2)
    return x1, x2, y1, y2


class AdjustEnemyPosition:

    def __init__(self):
        rospy.Subscriber("/guard/perception_of_evader", Odometry, self.listener_callback)
        self.pub = rospy.Publisher('/improved_position', Odometry, queue_size=5)

        self.enemy_pos_noise_x = []
        self.enemy_pos_noise_y = []

        self.enemy_pos_refactor_x_average = []
        self.enemy_pos_refactor_y_average = []

        self.enemy_pos_refactor_x_closer = []
        self.enemy_pos_refactor_y_closer = []

        self.enemy_pos_refactor_x_both = []
        self.enemy_pos_refactor_y_both = []

    def listener_callback(self, message):
        self.enemy_pos_noise_x.append(message.pose.pose.position.x)
        self.enemy_pos_noise_y.append(message.pose.pose.position.y)
        self.idea_get_points_closer_together()
        self.idea_average_points()
        self.idea_average_points_after_get_points_closer()
        odom = Odometry()
        if len(self.enemy_pos_refactor_x_both)>0:
            odom.pose.pose.position.x = self.enemy_pos_refactor_x_both[-1]
            odom.pose.pose.position.y = self.enemy_pos_refactor_y_both[-1]
            self.pub.publish(odom)


    def idea_get_points_closer_together(self):
        self.enemy_pos_refactor_x_closer = np.copy(self.enemy_pos_noise_x)
        self.enemy_pos_refactor_y_closer = np.copy(self.enemy_pos_noise_y)
        if len(self.enemy_pos_refactor_x_closer) > 1:
            self.enemy_pos_refactor_x_closer[-1], self.enemy_pos_refactor_x_closer[-2], self.enemy_pos_refactor_y_closer[
                -1], self.enemy_pos_refactor_y_closer[-2] = get_points_closer_together(
                self.enemy_pos_refactor_x_closer[-1], self.enemy_pos_refactor_x_closer[-2],
                self.enemy_pos_refactor_y_closer[-1], self.enemy_pos_refactor_y_closer[-2])

    def idea_average_points(self):
        if len(self.enemy_pos_noise_x) > 1:
            self.enemy_pos_refactor_x_average.append((self.enemy_pos_noise_x[-2] + self.enemy_pos_noise_x[-1]) / 2)
            self.enemy_pos_refactor_y_average.append((self.enemy_pos_noise_y[-2] + self.enemy_pos_noise_y[-1]) / 2)


    def idea_average_points_after_get_points_closer(self):
        if len(self.enemy_pos_noise_x) > 1:
            self.enemy_pos_refactor_x_both.append(
                (self.enemy_pos_refactor_x_closer[-2] + self.enemy_pos_refactor_x_closer[-1]) / 2)
            self.enemy_pos_refactor_y_both.append(
                (self.enemy_pos_refactor_y_closer[-2] + self.enemy_pos_refactor_y_closer[-1]) / 2)




if __name__ == '__main__':
    rospy.init_node('point_improvement')
    AdjustEnemyPosition()
    rospy.spin()
