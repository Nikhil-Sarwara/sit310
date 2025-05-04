#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, WheelEncoderStamped
import time

### ===============================
### Robot Configuration & Constants
### ===============================

CMD_TOPIC = '/majdoor/car_cmd_switch_node/cmd'
FSM_TOPIC = '/majdoor/fsm_node/mode'
RIGHT_ENCODER_TOPIC = '/majdoor/right_wheel_encoder_node/tick'
LEFT_ENCODER_TOPIC = '/majdoor/left_wheel_encoder_node/tick'

TURN_TICKS_THRESHOLD = 40

MODE_LANE_FOLLOWING = 'LANE_FOLLOWING'

STAGE_IDLE = 'idle'
STAGE_TURNING = 'turning'
STAGE_DONE = 'done'

### ===============================
### Robot Class (Tracks State)
### ===============================

class Robot:
    def __init__(self):
        self.right_ticks = 0
        self.left_ticks = 0

        self.start_right_ticks = 0
        self.start_left_ticks = 0

        self.current_mode = None
        self.stage = STAGE_IDLE

    def update_right_ticks(self, tick_count):
        self.right_ticks = tick_count

    def update_left_ticks(self, tick_count):
        self.left_ticks = tick_count

    def set_start_ticks(self):
        self.start_right_ticks = self.right_ticks
        self.start_left_ticks = self.left_ticks

    def get_right_tick_diff(self):
        return abs(self.right_ticks - self.start_right_ticks)

    def get_left_tick_diff(self):
        return abs(self.left_ticks - self.start_left_ticks)

    def update_mode(self, mode):
        self.current_mode = mode

    def is_in_lane_following(self):
        return self.current_mode == MODE_LANE_FOLLOWING

    def set_stage(self, stage):
        self.stage = stage

    def get_stage(self):
        return self.stage

    def is_turn_complete(self):
        return self.get_right_tick_diff() >= TURN_TICKS_THRESHOLD

    def display_status(self):
        print("\n========= Robot Status =========")
        print(f" FSM Mode         : {self.current_mode}")
        print(f" Stage            : {self.stage}")
        print(f" Right Encoder    : {self.right_ticks} (Δ {self.get_right_tick_diff()})")
        print(f" Left Encoder     : {self.left_ticks} (Δ {self.get_left_tick_diff()})")
        print("================================\n")

### ===============================
### Robot Controller (ROS Interface)
### ===============================

class RobotController:
    def __init__(self, control_enabled=True):
        rospy.init_node('closed_loop_virtual_robot', anonymous=True)

        self.robot = Robot()
        self.control_enabled = control_enabled

        self.cmd_pub = rospy.Publisher(CMD_TOPIC, Twist2DStamped, queue_size=1)

        rospy.Subscriber(FSM_TOPIC, FSMState, self.fsm_callback, queue_size=1)
        rospy.Subscriber(RIGHT_ENCODER_TOPIC, WheelEncoderStamped, self.right_encoder_callback, queue_size=1)
        rospy.Subscriber(LEFT_ENCODER_TOPIC, WheelEncoderStamped, self.left_encoder_callback, queue_size=1)

        self.cmd_msg = Twist2DStamped()
        rospy.loginfo("Virtual Robot Controller Initialized")

        self.status_display_enabled = True  # Always display robot status

    def fsm_callback(self, msg):
        self.robot.update_mode(msg.state)

    def right_encoder_callback(self, msg):
        self.robot.update_right_ticks(msg.data)
        if self.status_display_enabled:
            self.robot.display_status()
        if self.control_enabled:
            self.control_logic()

    def left_encoder_callback(self, msg):
        self.robot.update_left_ticks(msg.data)

    def control_logic(self):
        stage = self.robot.get_stage()

        if stage == STAGE_IDLE and self.robot.is_in_lane_following():
            self.robot.set_start_ticks()
            self.robot.set_stage(STAGE_TURNING)
            self.send_command(0.0, -3.0)
            rospy.loginfo("Started in-place turn")

        elif stage == STAGE_TURNING:
            if self.robot.is_turn_complete():
                self.stop_robot()
                self.robot.set_stage(STAGE_DONE)
                rospy.loginfo("Turn complete")

    def send_command(self, velocity, omega):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = velocity
        self.cmd_msg.omega = omega
        self.cmd_pub.publish(self.cmd_msg)

    def stop_robot(self):
        self.send_command(0.0, 0.0)

    def move_straight(self, distance_ticks):
        self.robot.set_start_ticks()
        self.robot.set_stage(STAGE_TURNING)
        self.send_command(0.5, 0.0)  # Move straight at 0.5 m/s
        rospy.loginfo("Moving straight...")
        rospy.sleep(2)
        self.stop_robot()

    def turn_90_degrees(self):
        self.robot.set_start_ticks()
        self.robot.set_stage(STAGE_TURNING)
        self.send_command(0.0, -3.0)  # In-place right turn
        rospy.loginfo("Turning 90 degrees...")
        rospy.sleep(2)
        self.stop_robot()

    def make_square(self):
        for _ in range(4):
            self.move_straight(50)  # Move straight for 50 ticks
            rospy.sleep(1)
            self.turn_90_degrees()
            rospy.sleep(1)

    def control_main_menu(self):
        while True:
            print("\n==== Main Menu ====")
            print("1. Move Robot Straight")
            print("2. Turn Robot 90 Degrees")
            print("3. Make Robot Draw a Square")
            print("4. Exit")
            choice = input("Enter your choice: ").strip()

            if choice == "1":
                self.move_straight(50)  # Move for 50 ticks
            elif choice == "2":
                self.turn_90_degrees()
            elif choice == "3":
                self.make_square()
            elif choice == "4":
                print("Exiting...")
                break
            else:
                print("Invalid choice. Please select again.")

### ===============================
### Main Program (Interactive Menu)
### ===============================

def main():
    try:
        controller = RobotController(control_enabled=True)  # Enable control mode
        controller.control_main_menu()
    except rospy.ROSInterruptException:
        print("ROS node interrupted.")

if __name__ == '__main__':
    main()


