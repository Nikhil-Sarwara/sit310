#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, FSMState

class RobotControl:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('move_robot_backward_ticks', anonymous=True)

        # Create publishers and subscribers
        self.cmd_pub = rospy.Publisher('/majdoor/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.right_encoder_ticks = 0
        self.left_encoder_ticks = 0
        self.initial_right_ticks = 0
        self.initial_left_ticks = 0
        self.fsm_state = None

        # Initialize the Twist2DStamped message
        self.cmd_msg = Twist2DStamped()

        # Set up subscribers to get encoder feedback
        rospy.Subscriber('/majdoor/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_encoder_callback)
        rospy.Subscriber('/majdoor/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_encoder_callback)
        rospy.Subscriber('/majdoor/fsm_node/mode', FSMState, self.fsm_callback)

    def right_encoder_callback(self, msg):
        self.right_encoder_ticks = msg.data

    def left_encoder_callback(self, msg):
        self.left_encoder_ticks = msg.data

    def fsm_callback(self, msg):
        self.fsm_state = msg.state

    def wait_for_lane_following(self):
        rospy.loginfo("Waiting for lane following mode...")
        while not rospy.is_shutdown():
            if self.fsm_state == "LANE_FOLLOWING":
                rospy.loginfo("Robot is now in LANE_FOLLOWING mode.")
                break
            rospy.sleep(1)  # Sleep and check again

    def move_backward(self, target_ticks):
        # Wait until robot is in lane following mode
        self.wait_for_lane_following()

        # Store initial encoder values before starting movement
        self.initial_right_ticks = self.right_encoder_ticks
        self.initial_left_ticks = self.left_encoder_ticks

        # Set the robot's velocity to move backward (linear velocity < 0)
        self.cmd_msg.v = -0.5  # Move backward at 0.5 m/s
        self.cmd_msg.omega = 0.0  # No rotation (move straight)

        # Publish the command to start moving
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_pub.publish(self.cmd_msg)

        rospy.loginfo("Started moving backward...")

        # Wait a bit to ensure the robot starts moving and encoder values are being updated
        rospy.sleep(1)  # Give the robot time to start moving

        # Keep moving until we've reached the target ticks
        while not rospy.is_shutdown():
            # Calculate the total ticks moved since the initial position
            right_ticks_moved = self.right_encoder_ticks - self.initial_right_ticks
            left_ticks_moved = self.left_encoder_ticks - self.initial_left_ticks

            # Calculate average movement in both wheels
            total_ticks_moved = (right_ticks_moved + left_ticks_moved) / 2

            rospy.loginfo("Total ticks moved: %d (Right: %d, Left: %d)", total_ticks_moved, right_ticks_moved, left_ticks_moved)

            # Check if the robot has moved the desired number of ticks
            if total_ticks_moved <= -target_ticks:
                rospy.loginfo("Target reached: %d ticks", target_ticks)
                break  # Stop the movement

            rospy.sleep(0.1)  # Sleep for a brief moment to avoid high CPU usage

        # Stop the robot after reaching the target ticks
        self.stop_robot()

    def stop_robot(self):
        # Stop the robot by setting velocity to zero
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.cmd_pub.publish(self.cmd_msg)
        rospy.loginfo("Robot stopped.")

if __name__ == '__main__':
    try:
        # Instantiate the RobotControl class and move backward 150 ticks
        robot_control = RobotControl()
        robot_control.move_backward(150)
    except rospy.ROSInterruptException:
        pass


