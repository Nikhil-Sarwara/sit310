#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import WheelEncoderStamped

class Drive_Square:

    def __init__(self):
        self.cmd_msg = Twist2DStamped()

        self.start_left_ticks = None
        self.current_left_ticks = 0
        self.last_left_sync = 0

        self.start_right_ticks = None
        self.current_right_ticks = 0
        self.last_right_sync = 0

        self.moving = False
        self.robot_paused = False  # If robot is temporarily paused for sync

        rospy.init_node('drive_square_node', anonymous=True)

        self.pub = rospy.Publisher('/majdoor/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/majdoor/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        rospy.Subscriber('/majdoor/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_encoder_callback, queue_size=1)
        rospy.Subscriber('/majdoor/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_encoder_callback, queue_size=1)

    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
            self.moving = False
            self.start_left_ticks = None
            self.start_right_ticks = None
        elif msg.state == "LANE_FOLLOWING":
            self.start_left_ticks = None
            self.start_right_ticks = None
            self.last_left_sync = 0
            self.last_right_sync = 0
            self.moving = True
            self.robot_paused = False

    def left_encoder_callback(self, msg):
        self.current_left_ticks = msg.data
        self.check_and_update()

    def right_encoder_callback(self, msg):
        self.current_right_ticks = msg.data
        self.check_and_update()

    def check_and_update(self):
        if not self.moving:
            return

        # Initialize starting ticks
        if self.start_left_ticks is None or self.start_right_ticks is None:
            self.start_left_ticks = self.current_left_ticks
            self.start_right_ticks = self.current_right_ticks
            rospy.loginfo("Started movement: Left=%d, Right=%d", self.start_left_ticks, self.start_right_ticks)
            self.send_forward_command()
            return

        # Calculate differences
        left_diff = abs(self.current_left_ticks - self.start_left_ticks)
        right_diff = abs(self.current_right_ticks - self.start_right_ticks)

        # Final stop condition
        if left_diff >= 600 and right_diff >= 600:
            rospy.loginfo("Both wheels reached 600 ticks. Stopping.")
            self.stop_robot()
            self.moving = False
            return

        # Only continue checking sync logic if not paused due to desync
        if not self.robot_paused:
            # Every 10 ticks from each side, check if desynced
            if (left_diff - self.last_left_sync >= 10) and (right_diff - self.last_right_sync >= 10):
                tick_gap = abs(left_diff - right_diff)
                rospy.loginfo("Sync check: Left=%d, Right=%d, Gap=%d", left_diff, right_diff, tick_gap)

                if tick_gap > 0:
                    rospy.loginfo("Wheel desync detected. Pausing movement.")
                    self.stop_robot()
                    self.robot_paused = True
                else:
                    self.last_left_sync = left_diff
                    self.last_right_sync = right_diff

        else:
            # If paused, check if the slower wheel has caught up
            left_diff = abs(self.current_left_ticks - self.start_left_ticks)
            right_diff = abs(self.current_right_ticks - self.start_right_ticks)
            if left_diff == right_diff:
                rospy.loginfo("Wheels resynced at %d ticks. Resuming movement.", left_diff)
                self.last_left_sync = left_diff
                self.last_right_sync = right_diff
                self.send_forward_command()
                self.robot_paused = False

    def send_forward_command(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.5
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass

