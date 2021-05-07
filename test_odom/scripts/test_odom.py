#!/usr/bin/env python


import rospy
from tf import transformations


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from math import pi, sin, cos


class TestOdom:
    def __init__(self):
        self.is_odom = False
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        cmd_vel = Twist()
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        freq = 50
        rate = rospy.Rate(freq)

        while not rospy.is_shutdown():
            if self.is_odom:
                linear_position = self.odom_msg.pose.pose.position.x
                qx, qy, qz, qw =  self.odom_msg.pose.pose.orientation.x, self.odom_msg.pose.pose.orientation.y, self.odom_msg.pose.pose.orientation.z, self.odom_msg.pose.pose.orientation.w
                (_, _, angular_pos) = transformations.euler_from_quaternion([qx,qy,qz,qw])

                input_linear_vel = 0.5 # m/s
                input_angular_vel = 20 * pi / 180 # degree/sec

                
                ## test linear
                # if linear_position < 10:
                #     cmd_vel.linear.x = input_linear_vel
                #     print("moving")
                # else:
                #     cmd_vel.linear.x = 0.


                ## test angular
                if angular_pos < pi/2:
                    cmd_vel.angular.z = input_angular_vel
                    print(angular_pos)
                    # print("spinning")
                else:
                    cmd_vel.angular.z = 0.

                self.cmd_pub.publish(cmd_vel)

            rate.sleep()

    def odom_callback(self, msg):
        self.is_odom = True
        self.odom_msg = msg


if __name__ == "__main__":
    rospy.init_node("scout_odometry", anonymous=True)
    test_odom = TestOdom()

    rospy.spin()