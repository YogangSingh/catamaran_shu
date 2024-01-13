#!/usr/bin/env python
'''
Defines a circular controller controller to follow a path
with radius of 5.

'''


import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import math
import tf

class CircularBehaviorController:
    def __init__(self):
        rospy.init_node('path_following_controller')

        # Create a publisher for sending control commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Publisher for the generated path
        self.generated_path_pub = rospy.Publisher('/circular_behavior_generated_path', Path, queue_size=10)
        
        # Publisher for the followed path
        self.followed_path_pub = rospy.Publisher('/circular_behavior_followed_path', Path, queue_size=10)

        # Subscribe to odometry topic (assuming '/odom' here, adjust based on your system)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)

        # Initialize other variables and parameters
        self.current_path = None
        self.path_index = 0
        self.Kp = 1.0  # Proportional gain for steering control
        self.L = 0.31  # Wheelbase of the vehicle (distance between front and rear axles)
        self.trajectory = []  # List to store the followed trajectory

    def odom_callback(self, odom_msg):
        # Process odometry data and update internal state

        # Extract current pose information (assuming Pose information is needed)
        current_pose = odom_msg.pose.pose

        # Generate circular path based on current position
        if self.current_path is None:
            self.current_path = self.generate_circular_path((current_pose.position.x, current_pose.position.y), radius=5.0, num_points=100)
            
            # Publish the generated path
            self.publish_generated_path()

        # Implement logic to follow the circular path using the Stanley controller
        self.follow_path(current_pose)

    def follow_path(self, current_pose):
        if self.current_path is not None and self.path_index < len(self.current_path):
            # Get the next waypoint from the path
            target_point = self.current_path[self.path_index]

            # Calculate heading error and cross-track error (CTE)
            heading_error = math.atan2(target_point[1] - current_pose.position.y, target_point[0] - current_pose.position.x)
            cte = self.calculate_cte(target_point, current_pose)

            # Calculate the desired steering angle using the Stanley controller
            steering_angle = heading_error + math.atan2(self.Kp * cte, 1.0)

            # Calculate differential wheel velocities for a differential drive system
            v = 1.0  # Constant forward velocity (adjust as needed)
            wheelbase = self.L

            left_wheel_velocity = v - 0.5 * steering_angle * wheelbase
            right_wheel_velocity = v + 0.5 * steering_angle * wheelbase

            # Create a Twist message with linear and angular velocities
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = (left_wheel_velocity + right_wheel_velocity) / 2.0
            cmd_vel_msg.angular.z = (right_wheel_velocity - left_wheel_velocity) / wheelbase

            # Publish the control commands
            self.cmd_vel_pub.publish(cmd_vel_msg)

            # Update the trajectory with the current position
            self.trajectory.append((current_pose.position.x, current_pose.position.y))

            # Check if the robot is close to the current waypoint and update the index
            distance_to_waypoint = math.sqrt((target_point[0] - current_pose.position.x)**2 + (target_point[1] - current_pose.position.y)**2)
            if distance_to_waypoint < 0.1:  # Adjust the threshold based on your requirements
                self.path_index += 1

            # Publish the followed path
            self.publish_followed_path()

    def calculate_cte(self, target_point, current_pose):
        # Calculate cross-track error (lateral distance between the vehicle and the desired path)
        cte = ((current_pose.position.x - target_point[0]) * (target_point[1] - current_pose.position.y) -
               (target_point[0] - current_pose.position.x) * (current_pose.position.y - target_point[1])) / \
              math.sqrt((target_point[0] - current_pose.position.x)**2 + (target_point[1] - current_pose.position.y)**2)

        return cte

    def generate_circular_path(self, center, radius, num_points):
        path = Path()
        path.header.frame_id = "map"  # Adjust frame_id as needed

        for i in range(num_points):
            theta = i * (2 * math.pi / num_points)
            x = center[0] + radius * math.cos(theta)
            y = center[1] + radius * math.sin(theta)

            # Create a PoseStamped for each point with orientation information
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y

            # Calculate quaternion for the orientation (e.g., facing forward)
            quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
            pose_stamped.pose.orientation.x = quaternion[0]
            pose_stamped.pose.orientation.y = quaternion[1]
            pose_stamped.pose.orientation.z = quaternion[2]
            pose_stamped.pose.orientation.w = quaternion[3]

            # Add the PoseStamped to the Path
            path.poses.append(pose_stamped)

        return path


    def publish_generated_path(self):
        # Create a Path message for the generated path
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'odom'  # Adjust the frame_id as needed

        # Add each waypoint to the path
        for point in self.current_path:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.w = 1.0  # No rotation for simplicity
            path_msg.poses.append(pose)

        # Publish the generated path
        self.generated_path_pub.publish(path_msg)

    def publish_followed_path(self):
        # Create a Path message for the followed path
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'odom'  # Adjust the frame_id as needed

        # Add each point in the trajectory to the path
        for point in self.trajectory:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.w = 1.0  # No rotation for simplicity
            path_msg.poses.append(pose)

        # Publish the followed path
        self.followed_path_pub.publish(path_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    controller = CircularBehaviorController()
    controller.run()
