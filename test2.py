#!/usr/bin/env python

import rospy
import copy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class Base(object):
    def __init__(self):
        self._odom_sub = rospy.Subscriber('odom', nav_msgs.msg.Odometry, callback=self._odom_callback)


    def _odom_callback(self, msg):
        self.current_odom = msg

    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.
        Args:
            distance (float): The distance, in meters, to move.
            speed (float): The speed to travel, in meters/second.
        """
        # Wait until the base has received at least one message on /odom
        while self.current_odom is None:
            rospy.sleep(0.1)
        
        # Record start position using deepcopy to avoid referencing the same object
        start = copy.deepcopy(self.current_odom)
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            if self.current_odom is None:
                continue

            # Calculate the current displacement
            dx = self.current_odom.pose.pose.position.x - start.pose.pose.position.x
            dy = self.current_odom.pose.pose.position.y - start.pose.pose.position.y
            current_distance = (dx ** 2 + dy ** 2) ** 0.5  # Euclidean distance

            # Check if the robot has traveled the desired distance
            if (distance > 0 and current_distance >= distance) or (distance < 0 and current_distance <= distance):
                break

            # Determine the direction of travel
            direction = -1 if distance < 0 else 1
            self.move(direction * speed, 0)
            rate.sleep()
            
        # Stop the robot after moving the desired distance
        self.move(0, 0)

    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance (float): The angle, in radians, to rotate. A positive value rotates counter-clockwise.
            speed (float): The angular speed to rotate, in radians/second.
        """
        # Normalize angular distance to the range [0, 2*pi)
        angular_distance = angular_distance % (2 * math.pi)
        # Wait until the base has received at least one message on /odom
        while self.current_odom is None:
            rospy.sleep(0.1)

        # Record start orientation from odometry
        start_yaw = self.get_yaw_from_odom(self.current_odom)
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if self.current_odom is None:
                continue

            current_yaw = self.get_yaw_from_odom(self.current_odom)
            # Compute the difference in yaw
            yaw_diff = (current_yaw - start_yaw) % (2 * math.pi)

            # Determine if the remaining angle to turn is within tolerance
            if abs(yaw_diff - angular_distance) < 0.01:
                break

            # Determine the direction of rotation
            direction = -1 if angular_distance < 0 else 1
            self.move(0, direction * speed)
            rate.sleep()

        # Stop the robot after rotation
        self.move(0, 0)

    def move(self, linear_speed, angular_speed):
        """Send velocity commands to the robot."""
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self._vel_pub.publish(twist)

    def get_yaw_from_odom(self, odom):
        """Extracts the yaw angle from an odometry message."""
        quaternion = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)
        return yaw

def euler_from_quaternion(quat):
    """Convert quaternion (x, y, z, w) to euler angles."""
    x, y, z, w = quat
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z

if __name__ == '__main__':
    base = Base()
    rospy.spin()
