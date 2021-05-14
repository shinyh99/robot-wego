#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

from math import pi, sin, cos

class GoalPub:
    def __init__(self):
        self.goal_pub = rospy.Publisher("/move_base_simple", PoseStamped, queue_size=1)
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"