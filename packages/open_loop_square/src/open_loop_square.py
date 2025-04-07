#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
import math

class Drive_Square:
    def __init__(self):
        #Initialize global class variables
        self.cmd_msg = Twist2DStamped()

        #Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)

        #Initialize Pub/Subs
        self.pub = rospy.Publisher('/majdoor/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/majdoor/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)

        # Define parameters for the square motion
        self.linear_speed = 0.2  # Adjust as needed
        self.forward_duration = 2.0  # Adjust as needed
        self.angular_speed = 0.8  # Adjust as needed (positive for counter-clockwise)
        self.turn_duration = math.pi / 2 / self.angular_speed # Time to turn 90 degrees

        self.moving = False # Flag to control the movement sequence
        self.side_count = 0 # Counter for the number of sides completed

    # robot only moves when lane following is selected on the duckiebot joystick app
    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
            self.moving = False
            self.side_count = 0
        elif msg.state == "LANE_FOLLOWING" and not self.moving:
            rospy.sleep(1) # Wait for a sec for the node to be ready
            self.moving = True
            self.side_count = 0
            self.move_robot()

    # Sends zero velocities to stop the robot
    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Stopped!")

    # Spin forever but listen to message callbacks
    def run(self):
        rospy.spin() # keeps node from exiting until node has shutdown

    # Robot drives in a square and then stops
    def move_robot(self):
        if not self.moving:
            return

        if self.side_count < 4:
            rospy.loginfo(f"Moving forward - Side {self.side_count + 1}")
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = self.linear_speed
            self.cmd_msg.omega = 0.0
            self.pub.publish(self.cmd_msg)
            rospy.sleep(self.forward_duration)

            rospy.loginfo(f"Turning - Side {self.side_count + 1}")
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = self.angular_speed
            self.pub.publish(self.cmd_msg)
            rospy.sleep(self.turn_duration)

            self.side_count += 1
            # Call move_robot again to continue the sequence
            rospy.Timer(rospy.Duration(0.1), self.timer_callback, oneshot=True)
        else:
            self.stop_robot()
            self.moving = False
            self.side_count = 0
            rospy.loginfo("Square completed!")

    def timer_callback(self, event):
        self.move_robot()

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass
