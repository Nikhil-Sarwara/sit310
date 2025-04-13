#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, WheelEncoderStamped

class Drive_Square:

    def __init__(self):
        self.cmd_msg = Twist2DStamped()

        self.start_right_ticks = 0
        self.current_right_ticks = 0

        self.stage = "idle"  # idle, turning, done

        rospy.init_node('drive_square_node', anonymous=True)

        self.pub = rospy.Publisher('/majdoor/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/majdoor/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        rospy.Subscriber('/majdoor/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_encoder_callback, queue_size=1)

    def fsm_callback(self, msg):
        rospy.loginfo("FSM State: %s", msg.state)
        if msg.state == "LANE_FOLLOWING" and self.stage == "idle":
            self.stage = "turning"
            self.start_right_ticks = self.current_right_ticks
            self.send_command(0.0, -3.0)  # In-place right turn
            rospy.loginfo("Turning right...")

    def right_encoder_callback(self, msg):
        self.current_right_ticks = msg.data

        if self.stage == "turning":
            tick_diff = abs(self.current_right_ticks - self.start_right_ticks)
            rospy.loginfo("Right tick diff: %d", tick_diff)

            if tick_diff >= 40:
                self.stop_robot()
                self.stage = "done"
                rospy.loginfo("Reached 40 ticks on right wheel. Turn complete.")

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

