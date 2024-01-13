#!/usr/bin/env python
'''
Improves the circular controller with a full PID controller, dynamic replanning, and safety mechanisms.
'''

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import math
import tf

# A simple PID control class
class PIDController:
    def __init__(self, Kp, Ki, Kd, max_integral, min_output, max_output):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_integral = max_integral
        self.min_output = min_output
        self.max_output = max_output
        self.reset()

    def reset(self):
        self.integral = 0
        self.previous_error = 0

    def control(self, error, delta_time):
        self.integral += error * delta_time
        # Anti-windup clamping
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
        derivative = (error - self.previous_error) / delta_time if delta_time > 0 else 0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        # Output clamping
        output = max(min(output, self.max_output), self.min_output)
        self.previous_error = error
        return output
   

class CircularMotionController:
    def __init__(self):
        rospy.init_node('circular_motion_controller', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.generated_path_pub = rospy.Publisher('/circular_motion_generated_path', Path, queue_size=10)
        self.followed_path_pub = rospy.Publisher('/circular_motion_followed_path', Path, queue_size=10)
        rospy.Subscriber('/odom/nav', Odometry, self.odom_callback)

        self.current_path = None
        self.path_index = 0
        self.heading_pid = PIDController(Kp=0.5, Ki=0.1, Kd=0.05, max_integral=1.0, min_output=-0.5, max_output=0.5)
        self.cte_pid = PIDController(Kp=1.0, Ki=0.05, Kd=0.02, max_integral=0.5, min_output=-1.0, max_output=1.0)
        self.last_time = rospy.Time.now()
        self.trajectory = []
        self.prev_linear_velocity = 0.0
        self.prev_angular_velocity = 0.0

   
    def odom_callback(self, odom_msg):
        current_pose = odom_msg.pose.pose
        if self.current_path is None:
            center = (current_pose.position.x, current_pose.position.y)
            radius = 0.00005  # Set the radius of the circle
            self.current_path = self.generate_circular_path(center, radius, num_points=100)

            self.publish_generated_path()
        self.follow_path(current_pose)

    def follow_path(self, current_pose):
        if self.current_path and self.path_index < len(self.current_path.poses):
            target_point = self.current_path.poses[self.path_index].pose.position
            current_time = rospy.Time.now()
            delta_time = (current_time - self.last_time).to_sec()
            self.last_time = current_time
            heading_error = math.atan2(target_point.y - current_pose.position.y, target_point.x - current_pose.position.x)
            cte = self.calculate_cte(current_pose)
            heading_pid_output = self.heading_pid.control(heading_error, delta_time)
            cte_pid_output = self.cte_pid.control(cte, delta_time)

            v = 10.0  # Desired velocity
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = v + cte_pid_output
            cmd_vel_msg.angular.z = heading_pid_output

            # Apply safety limits and publish velocities
            cmd_vel_msg.linear.x, cmd_vel_msg.angular.z = self.apply_safety_limits(cmd_vel_msg.linear.x, cmd_vel_msg.angular.z)
            self.cmd_vel_pub.publish(cmd_vel_msg)

            # Update trajectory for visualization
            self.trajectory.append(current_pose.position)

            # Check if we've reached the current target point
            distance_to_waypoint = math.sqrt((target_point.x - current_pose.position.x)**2 + (target_point.y - current_pose.position.y)**2)
            if distance_to_waypoint < 0.1:
                self.path_index += 1
            self.publish_followed_path()

    def calculate_cte(self, current_pose):
        if self.path_index == 0 or self.path_index >= len(self.current_path.poses) - 1:
            return 0  # No CTE when at start or end of path

        path_start = self.current_path.poses[self.path_index - 1].pose.position
        path_end = self.current_path.poses[self.path_index].pose.position

        path_vector = [path_end.x - path_start.x, path_end.y - path_start.y]
        path_length = math.sqrt(path_vector[0]**2 + path_vector[1]**2)
        path_unit_vector = [path_vector[0] / path_length, path_vector[1] / path_length]

        position_vector = [current_pose.position.x - path_start.x, current_pose.position.y - path_start.y]
        projection_length = position_vector[0] * path_unit_vector[0] + position_vector[1] * path_unit_vector[1]
        projection = [path_unit_vector[0] * projection_length, path_unit_vector[1] * projection_length]

        cte_vector = [position_vector[0] - projection[0], position_vector[1] - projection[1]]
        cte = math.sqrt(cte_vector[0]**2 + cte_vector[1]**2)

        cross_product = path_vector[0] * position_vector[1] - path_vector[1] * position_vector[0]
        cte *= math.copysign(1, cross_product)

        return cte
   
    def generate_circular_path(self, center, radius, num_points):
        path = Path()
        path.header.frame_id = "map"
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)

            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y

            quaternion = tf.transformations.quaternion_from_euler(0, 0, angle)
            pose_stamped.pose.orientation.x = quaternion[0]
            pose_stamped.pose.orientation.y = quaternion[1]
            pose_stamped.pose.orientation.z = quaternion[2]
            pose_stamped.pose.orientation.w = quaternion[3]

            path.poses.append(pose_stamped)

        return path
   
    def apply_safety_limits(self, linear_velocity, angular_velocity):
        max_linear_velocity = 10.5
        min_linear_velocity = -20.5
        max_angular_velocity = 0.5

        linear_velocity = max(min(linear_velocity, max_linear_velocity), min_linear_velocity)
        angular_velocity = max(min(angular_velocity, max_angular_velocity), -max_angular_velocity)

        linear_jump_threshold = 5.0
        angular_jump_threshold = 0.2

        if abs(linear_velocity - self.prev_linear_velocity) > linear_jump_threshold:
            linear_velocity = self.prev_linear_velocity

        if abs(angular_velocity - self.prev_angular_velocity) > angular_jump_threshold:
            angular_velocity = self.prev_angular_velocity

        self.prev_linear_velocity = linear_velocity
        self.prev_angular_velocity = angular_velocity

        return linear_velocity, angular_velocity
   
    def run(self):
        rospy.spin()

def main():
    try:
        controller = CircularMotionController()
        print("Circular behaviour node is starting.")
        rospy.spin()  # Keep the node running until interrupted
    except KeyboardInterrupt:
        print("Shutting down ROS Node due to keyboard interrupt.")
        rospy.signal_shutdown("KeyboardInterrupt")

if __name__ == '__main__':
    main()