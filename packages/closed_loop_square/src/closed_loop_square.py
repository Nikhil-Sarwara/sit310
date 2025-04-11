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
            rospy.loginfo("Starting movement...")

    def left_encoder_callback(self, msg):
        self.current_left_ticks = msg.data
        self.check_and_update()

    def right_encoder_callback(self, msg):
        self.current_right_ticks = msg.data
        self.check_and_update()

    def check_and_update(self):
        if not self.moving:
            return

        if self.start_left_ticks is None or self.start_right_ticks is None:
            self.start_left_ticks = self.current_left_ticks
            self.start_right_ticks = self.current_right_ticks
            rospy.loginfo("Initial ticks — Left: %d, Right: %d", self.start_left_ticks, self.start_right_ticks)
            self.send_command(0.5, 0.0)  # Start moving straight
            return

        left_diff = abs(self.current_left_ticks - self.start_left_ticks)
        right_diff = abs(self.current_right_ticks - self.start_right_ticks)

        # Stop condition
        if left_diff >= 600 and right_diff >= 600:
            rospy.loginfo("Both wheels reached 600 ticks. Stopping.")
            self.stop_robot()
            self.moving = False
            return

        # Sync check every 10 ticks
        if (left_diff - self.last_left_sync >= 10) and (right_diff - self.last_right_sync >= 10):
            rospy.loginfo("Tick check — Left: %d, Right: %d", left_diff, right_diff)
            tick_gap = left_diff - right_diff

            # Adjust based on which side is ahead
            if abs(tick_gap) >= 1:
                if tick_gap > 0:
                    # Left is ahead, curve slightly right
                    rospy.loginfo("Left wheel ahead by %d ticks. Adjusting right.", tick_gap)
                    self.send_command(0.3, 0.5)
                elif tick_gap < 0:
                    # Right is ahead, curve slightly left
                    rospy.loginfo("Right wheel ahead by %d ticks. Adjusting left.", -tick_gap)
                    self.send_command(0.3, -0.5)
            else:
                # Wheels are synced, go straight
                rospy.loginfo("Wheels synced. Moving straight.")
                self.send_command(0.5, 0.0)

            # Update sync checkpoints
            self.last_left_sync = left_diff
            self.last_right_sync = right_diff

    def send_command(self, velocity, omega):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = velocity
        self.cmd_msg.omega = omega
        self.pub.publish(self.cmd_msg)

    def stop_robot(self):
        self.send_command(0.0, 0.0)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass

