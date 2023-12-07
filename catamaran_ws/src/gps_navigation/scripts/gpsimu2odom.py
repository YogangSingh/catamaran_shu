#!/usr/bin/env python
'''
Combines messages from GPS and IMU, then publishes to Odometry 
to create a integrated nav solution.

'''

import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

class Node():
    def __init__(self, pose_index=None, mstates_index=None, input_msg_type='Pose'):
        self.odommsg = Odometry()
        self.odompub = rospy.Publisher("odometry/nav", Odometry, queue_size=10)
        rospy.Subscriber("/fix", NavSatFix, self.gpscallback)
        rospy.Subscriber("/imu", Imu, self.imucallback)
        self.rxgps = False
        self.rximu = False
        self.lastimutime = rospy.get_time()

        print("Waiting for GPS and IMU connections...")

    def gpscallback(self, data):
        self.rxgps = True
        self.odommsg.pose.pose.position.x = data.longitude
        self.odommsg.pose.pose.position.y = data.latitude
        self.odommsg.pose.pose.position.z = data.altitude
        self.odommsg.pose.covariance[0] = data.position_covariance[0]
        self.odommsg.pose.covariance[7] = data.position_covariance[4]
        self.odommsg.pose.covariance[14] = data.position_covariance[8]

        print("GPS connected.")

    def imucallback(self, data):
        self.rximu = True
        self.odommsg.pose.pose.orientation = data.orientation
        self.odommsg.pose.covariance[21] = data.orientation_covariance[0]
        self.odommsg.pose.covariance[28] = data.orientation_covariance[4]
        self.odommsg.pose.covariance[35] = data.orientation_covariance[8]

        self.odommsg.twist.twist.angular = data.angular_velocity
        self.odommsg.twist.covariance[21] = data.angular_velocity_covariance[0]
        self.odommsg.twist.covariance[28] = data.angular_velocity_covariance[4]
        self.odommsg.twist.covariance[35] = data.angular_velocity_covariance[8]

        now = rospy.get_time()
        dt = now - self.lastimutime
        self.lastimutime = now
        self.odommsg.twist.twist.linear.x += data.linear_acceleration.x * dt
        self.odommsg.twist.twist.linear.y += data.linear_acceleration.y * dt
        self.odommsg.twist.twist.linear.z += data.linear_acceleration.z * dt
        self.odommsg.twist.covariance[0] = data.linear_acceleration_covariance[0]
        self.odommsg.twist.covariance[7] = data.linear_acceleration_covariance[4]
        self.odommsg.twist.covariance[14] = data.linear_acceleration_covariance[8]

        print("IMU connected.")

    def publishodom(self):
        if self.rximu and self.rxgps:
            self.odompub.publish(self.odommsg)
            print("Publishing odometry.")

if __name__ == '__main__':
    rospy.init_node('gpsimu2odom', anonymous=True)
    node = Node()
    r = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        node.publishodom()
        r.sleep()