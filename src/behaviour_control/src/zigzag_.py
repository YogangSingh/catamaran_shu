#!/usr/bin/env python
'''
Defines a zigzag controller controller

'''
#define global variable

global current_pose;

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import math
import tf

class ZigzagBehaviorController:
    def __init__(self):
        rospy.init_node('path_following_controller')

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.generated_path_pub = rospy.Publisher('/zigzag_behavior_generated_path', Path, queue_size=10)
        self.followed_path_pub = rospy.Publisher('/zigzag_behavior_followed_path', Path, queue_size=10)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)

        self.current_path = None
        self.path_index = 0
        self.Kp = 1.0
        self.L = 1.0
        self.trajectory = []

    def odom_callback(self, odom_msg):
        current_pose = odom_msg.pose.pose

        if self.current_path is None:
            num_segments = 5
            segment_length = 3.0
            self.current_path = self.generate_zigzag_path((current_pose.position.x, current_pose.position.y), num_segments, segment_length)
            self.publish_generated_path()

        self.follow_path(current_pose)

    def follow_path(self, current_pose):
        if self.current_path is not None and self.path_index < len(self.current_path):
            target_point = self.current_path[self.path_index]

            heading_error = math.atan2(target_point[1] - current_pose.position.y, target_point[0] - current_pose.position.x)
            cte = self.calculate_cte(target_point, current_pose)

            steering_angle = heading_error + math.atan2(self.Kp * cte, 1.0)

            v = 1.0
            wheelbase = self.L

            left_wheel_velocity = v - 0.5 * steering_angle * wheelbase
            right_wheel_velocity = v + 0.5 * steering_angle * wheelbase

            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = (left_wheel_velocity + right_wheel_velocity) / 2.0
            cmd_vel_msg.angular.z = (right_wheel_velocity - left_wheel_velocity) / wheelbase

            self.cmd_vel_pub.publish(cmd_vel_msg)

            self.trajectory.append((current_pose.position.x, current_pose.position.y))

            distance_to_waypoint = math.sqrt((target_point[0] - current_pose.position.x)**2 + (target_point[1] - current_pose.position.y)**2)
            if distance_to_waypoint < 0.1:
                self.path_index += 1

            self.publish_followed_path()

    def calculate_cte(self, target_point, current_pose):
        cte = ((current_pose.position.x - target_point[0]) * (target_point[1] - current_pose.position.y) -
               (target_point[0] - current_pose.position.x) * (current_pose.position.y - target_point[1])) / \
              math.sqrt((target_point[0] - current_pose.position.x)**2 + (target_point[1] - current_pose.position.y)**2)

        return cte

    def generate_zigzag_path(self, start, num_segments, segment_length):
        path = Path()
        path.header.frame_id = "map"

        for i in range(num_segments):
            if i % 2 == 0:
                end = (start[0] + segment_length, start[1])
            else:
                end = (start[0], start[1] + segment_length)
            path.poses.extend(self.generate_straight_line_path(start, end, num_points=50).poses)
            start = end

        return path

    def publish_generated_path(self):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'odom'

        for point in self.current_path:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.generated_path_pub.publish(path_msg)

    def publish_followed_path(self):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'odom'

        for point in self.trajectory:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.followed_path_pub.publish(path_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    controller = ZigzagBehaviorController()
    controller.run()
