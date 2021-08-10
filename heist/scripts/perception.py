#!/usr/bin/env python3
import rospy
import math
import sys
import numpy as np
from nav_msgs.msg import Odometry


class Perception:

    def __init__(self):
        self.source_topic = sys.argv[1]
        self.published_topic = sys.argv[2]
        self.position_noise = float(sys.argv[3]) 
        self.velocity_noise = float(sys.argv[4]) 

        self.percepted_odom = Odometry()

        rate = rospy.Rate(0.5)  # 0.5hz
        rospy.Subscriber(self.source_topic, Odometry, self.source_callback)
        self.pub = rospy.Publisher(self.published_topic, Odometry, queue_size=10)

        while not rospy.is_shutdown():
            self.pub.publish(self.percepted_odom)
            rate.sleep()

    def source_callback(self, odom):
        
        self.percepted_odom.pose.pose.position.x = odom.pose.pose.position.x + np.random.normal(0, self.position_noise)
        self.percepted_odom.pose.pose.position.y = odom.pose.pose.position.y + np.random.normal(0, self.position_noise)
        self.percepted_odom.twist.twist.linear.x = odom.twist.twist.linear.x + np.random.normal(0, self.velocity_noise) 
        self.percepted_odom.twist.twist.angular.z = odom.twist.twist.angular.z + np.random.normal(0, self.velocity_noise) 



if __name__ == '__main__':
    rospy.init_node("Perception")
    try:
        node = Perception()
    except rospy.ROSInterruptException:
        pass
