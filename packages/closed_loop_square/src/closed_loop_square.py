#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, WheelEncoderStamped

class Drive_Square:

    def __init__(self):
        self.cmd_msg = Twist2DStamped()

        self.start_left_ticks = 0
        self.current_left_ticks = 0
        self.last_left_sync = 0

        self.start_right_ticks = 0
        self.current_right_ticks = 0
        self.last_right_sync = 0

        self.stage = "idle"  # Can be "idle", "forward", "backward", "done"

        rospy.init_node('drive_square_node', anonymous=True)

        self.pub = rospy.Publisher('/majdoor/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/majdoor/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        rospy.Subscriber('/majdoor/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_encoder_callback, queue_size=1)
        rospy.Subscriber('/majdoor/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_encoder_callback, queue_size=1)

    def fsm_callback(self, msg):
        rospy.loginfo("FSM State: %s", msg.state)
        if msg.state == "LANE_FOLLOWING" and self.stage == "idle":
            self.stage = "forward"
            self.start_left_ticks = self.current_left_ticks
            self.start_right_ticks = self.current_right_ticks
            self.last_left_sync = 0
            self.last_right_sync = 0
            self.send_command(0.5, 0.0)
            rospy.loginfo("Starting forward movement...")

    def left_encoder_callback(self, msg):
        self.current_left_ticks = msg.data
        self.process_movement()

    def right_encoder_callback(self, msg):
        self.current_right_ticks = msg.data
        self.process_movement()

    def process_movement(self):
        if self.stage not in ["forward", "backward"]:
            return

        left_diff = abs(self.current_left_ticks - self.start_left_ticks)
        right_diff = abs(self.current_right_ticks - self.start_right_ticks)

        if left_diff >= 600 and right_diff >= 600:
            self.stop_robot()
            rospy.loginfo("%s movement complete.", self.stage.capitalize())
            if self.stage == "forward":
                rospy.sleep(1.0)
                self.stage = "backward"
                self.start_left_ticks = self.current_left_ticks
                self.start_right_ticks = self.current_right_ticks
                self.last_left_sync = 0
                self.last_right_sync = 0
                self.send_command(-0.5, 0.0)
                rospy.loginfo("Starting backward movement...")
            else:
                self.stage = "done"
                rospy.loginfo("All movements complete. Robot stopped.")
            return

        # Sync correction every 10 ticks
        left_sync = abs(self.current_left_ticks - self.start_left_ticks)
        right_sync = abs(self.current_right_ticks - self.start_right_ticks)

        if (left_sync - self.last_left_sync >= 10) and (right_sync - self.last_right_sync >= 10):
            tick_gap = left_sync - right_sync
            base_speed = 0.3 if self.stage == "forward" else -0.3

            if abs(tick_gap) >= 1:
                if tick_gap > 0:
                    self.send_command(base_speed, 0.5)
                else:
                    self.send_command(base_speed, -0.5)
            else:
                self.send_command(0.5 if self.stage == "forward" else -0.5, 0.0)

            self.last_left_sync = left_sync
            self.last_right_sync = right_sync

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


