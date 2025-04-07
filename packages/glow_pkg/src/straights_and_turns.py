#!/usr/bin/env python3

# Import Dependencies
import rospy
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import math

class TurtlesimStraightsAndTurns:
    def __init__(self):
        # Initialize class variables
        self.goal_distance = 0.0
        self.dist_goal_active = False
        self.forward_movement = True
        self.start_pose = Pose()

        self.goal_angle = 0.0
        self.angle_goal_active = False
        self.clockwise_rotation = True
        self.start_angle = 0.0

        self.goal_position = Point()
        self.pos_goal_active = False

        self.current_pose = Pose()

        # Initialize the node
        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        # Initialize subscribers
        rospy.Subscriber("/turtle_dist", Float64, self.distance_callback)
        rospy.Subscriber("/goal_angle", Float64, self.goal_angle_callback)
        rospy.Subscriber("/goal_distance", Float64, self.goal_distance_callback)
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)
        rospy.Subscriber("/goal_position", Point, self.goal_position_callback)

        # Initialize publisher
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Timer callback (acts like a loop)
        rospy.Timer(rospy.Duration(0.01), self.timer_callback)

        rospy.loginfo("Initialized node!")

        rospy.spin()

    def pose_callback(self, msg):
        self.current_pose = msg

    def distance_callback(self, msg):
        pass  # No longer used

    def goal_angle_callback(self, msg):
        self.goal_angle = abs(msg.data)
        self.angle_goal_active = True
        self.clockwise_rotation = msg.data < 0
        self.start_angle = self.current_pose.theta

    def goal_distance_callback(self, msg):
        self.goal_distance = abs(msg.data)
        self.dist_goal_active = True
        self.forward_movement = msg.data > 0
        self.start_pose = self.current_pose

    def goal_position_callback(self, msg):
        self.goal_position = msg
        self.pos_goal_active = True

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def timer_callback(self, event):
        twist_msg = Twist()

        if self.dist_goal_active:
            distance_tolerance = 0.05
            linear_speed = 0.5

            dx = self.current_pose.x - self.start_pose.x
            dy = self.current_pose.y - self.start_pose.y
            travelled = math.sqrt(dx ** 2 + dy ** 2)

            if travelled < self.goal_distance - distance_tolerance:
                twist_msg.linear.x = linear_speed if self.forward_movement else -linear_speed
            else:
                self.dist_goal_active = False
                twist_msg.linear.x = 0.0
                rospy.loginfo("Distance goal reached!")

            self.velocity_publisher.publish(twist_msg)

        elif self.angle_goal_active:
            angle_tolerance = 0.02
            angular_speed = 0.3

            turned = self.current_pose.theta - self.start_angle
            turned = self.normalize_angle(turned)
            target = self.goal_angle

            if abs(turned) < target - angle_tolerance:
                twist_msg.angular.z = -angular_speed if self.clockwise_rotation else angular_speed
            else:
                self.angle_goal_active = False
                twist_msg.angular.z = 0.0
                rospy.loginfo("Angle goal reached!")

            self.velocity_publisher.publish(twist_msg)

        elif self.pos_goal_active:
            position_tolerance = 0.1
            linear_speed = 0.4
            angular_speed = 0.2

            dx = self.goal_position.x - self.current_pose.x
            dy = self.goal_position.y - self.current_pose.y
            dist_to_goal = math.sqrt(dx ** 2 + dy ** 2)

            if dist_to_goal > position_tolerance:
                angle_to_goal = math.atan2(dy, dx)
                angle_diff = self.normalize_angle(angle_to_goal - self.current_pose.theta)

                if abs(angle_diff) > 0.1:
                    twist_msg.angular.z = angular_speed if angle_diff > 0 else -angular_speed
                    twist_msg.linear.x = 0.0
                else:
                    twist_msg.linear.x = linear_speed
                    twist_msg.angular.z = 0.0

                self.velocity_publisher.publish(twist_msg)
            else:
                self.pos_goal_active = False
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.velocity_publisher.publish(twist_msg)
                rospy.loginfo("Position goal reached!")

        else:
            # No goal active, stop
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.velocity_publisher.publish(twist_msg)

if __name__ == '__main__':
    try:
        TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException:
        pass




