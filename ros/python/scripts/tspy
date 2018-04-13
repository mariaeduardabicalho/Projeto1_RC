#!/usr/bin/env python


# Every python controller needs these lines
import roslib; roslib.load_manifest('lab1')
import rospy

# The velocity command message
from geometry_msgs.msg import Twist

# The laser scan message
from sensor_msgs.msg import LaserScan

# We use a hyperbolic tangent as a transfer function
from math import tanh


class Stopper:
    def __init__(self, distance, max_speed=0.1, min_speed=0.01):
        # How close should we get to things, and what's our maximum speed?
        self.distance = distance
        self.max_speed = max_speed
        self.min_speed = min_speed

        # Subscriber for the laser data
        self.sub = rospy.Subscriber('base_scan', LaserScan, self.laser_callback)

        # Publisher for movement commands
        self.pub = rospy.Publisher('cmd_vel', Twist)

        # Let the world know we're ready
        rospy.loginfo('Stopper initialized')

    def laser_callback(self, scan):
        # What's the closest laser reading
        closest = min(scan.ranges)

        # This is the command we send to the robot
        command = Twist()

        # If we're much more than 50cm away from things, then we want
        # to be going as fast as we can.  Otherwise, we want to slow
        # down.  A hyperbolic tangent transfer function will do this
        # nicely.
        command.linear.x = tanh(5 * (closest - self.distance)) * self.max_speed
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = 0.0

        # If we're going too slowly, then just stop
        if abs(command.linear.x) < self.min_speed:
            command.linear.x = 0

        rospy.logdebug('Distance: {0}, speed: {1}'.format(closest, command.linear.x))

        # Send the command to the motors
        self.pub.publish(command)


if __name__ == '__main__':
    rospy.init_node('stopper')

    # Get the distance from the parameter server.  Default is 0.5
    distance = rospy.get_param('distance', 0.5)

    # Set up the controller
    stopper = Stopper(distance)

    # Hand control over to ROS
    rospy.spin()
