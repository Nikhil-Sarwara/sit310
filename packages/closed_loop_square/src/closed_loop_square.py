#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
# Assuming a custom message for encoder data, you might need to create this or use an existing one
# For demonstration, let's assume a message type 'WheelEncoderStamped' with fields 'left_ticks' and 'right_ticks'
from std_msgs.msg import Int32  # Replace with your actual encoder message type

class Drive_Square:
    def __init__(self):
        # Robot parameters (replace with your robot's actual values)
        self.wheel_diameter = 0.05  # meters
        self.ticks_per_revolution = 1000
        self.wheel_separation = 0.1  # meters

        # Initialize global class variables
        self.cmd_msg = Twist2DStamped()
        self.left_ticks = 0
        self.right_ticks = 0
        self.initial_left_ticks = 0
        self.initial_right_ticks = 0

        # Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)

        # Initialize Pub/Subs
        self.pub = rospy.Publisher('/majdoor/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/majdoor/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        rospy.Subscriber('/majdoor/left_wheel_encoder', Int32, self.left_encoder_callback, queue_size=1)
        rospy.Subscriber('/majdoor/right_wheel_encoder', Int32, self.right_encoder_callback, queue_size=1)

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data

    # robot only moves when lane following is selected on the duckiebot joystick app
    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":
            rospy.sleep(1)  # Wait for a sec for the node to be ready
            # Example of how you might call the closed-loop functions
            # self.move_straight_closed_loop(1.0, 0.2)  # Move forward 1 meter at 0.2 m/s
            # rospy.sleep(5) # Give time to move
            # self.rotate_in_place_closed_loop(90.0, 0.5) # Rotate 90 degrees at 0.5 rad/s
            # rospy.sleep(5) # Give time to rotate
            self.move_robot() # Call the square drawing function

    # Sends zero velocities to stop the robot
    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Robot stopped.")

    def ticks_to_meters(self, ticks):
        """Converts encoder ticks to meters traveled by the wheel."""
        wheel_circumference = self.wheel_diameter * 3.14159
        meters_per_tick = wheel_circumference / self.ticks_per_revolution
        return ticks * meters_per_tick

    def meters_to_ticks(self, meters):
        """Converts meters to encoder ticks."""
        wheel_circumference = self.wheel_diameter * 3.14159
        ticks_per_meter = self.ticks_per_revolution / wheel_circumference
        return int(meters * ticks_per_meter)

    def degrees_to_radians(self, degrees):
        """Converts degrees to radians."""
        return degrees * 3.14159 / 180.0

    def radians_to_ticks(self, radians):
        """Converts radians of robot rotation to encoder ticks (difference between wheels)."""
        arc_length = (self.wheel_separation / 2) * radians
        return self.meters_to_ticks(arc_length)

    def move_straight_closed_loop(self, desired_distance, linear_speed):
        """Moves the robot straight for a desired distance using closed-loop control."""
        if linear_speed <= 0:
            rospy.logwarn("Linear speed should be positive for forward motion.")
            return

        self.initial_left_ticks = self.left_ticks
        self.initial_right_ticks = self.right_ticks
        target_ticks = self.meters_to_ticks(abs(desired_distance))
        rospy.loginfo(f"Moving straight for {desired_distance} meters (target ticks: {target_ticks}).")

        direction = 1 if desired_distance > 0 else -1
        self.cmd_msg.v = direction * linear_speed
        self.cmd_msg.omega = 0.0

        while not rospy.is_shutdown():
            current_left_ticks = self.left_ticks - self.initial_left_ticks
            current_right_ticks = self.right_ticks - self.initial_right_ticks
            distance_travelled = (self.ticks_to_meters(current_left_ticks) + self.ticks_to_meters(current_right_ticks)) / 2.0

            if distance_travelled >= abs(desired_distance):
                self.stop_robot()
                rospy.loginfo(f"Reached desired distance of {distance_travelled:.4f} meters.")
                break

            self.cmd_msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.cmd_msg)
            rospy.sleep(0.01) # Small delay for the loop

        return current_left_ticks, current_right_ticks

    def rotate_in_place_closed_loop(self, desired_rotation_degrees, angular_speed):
        """Rotates the robot in place for a desired angle using closed-loop control."""
        if angular_speed <= 0:
            rospy.logwarn("Angular speed should be positive.")
            return

        desired_rotation_radians = self.degrees_to_radians(desired_rotation_degrees)
        target_wheel_ticks = self.radians_to_ticks(abs(desired_rotation_radians))
        self.initial_left_ticks = self.left_ticks
        self.initial_right_ticks = self.right_ticks
        rospy.loginfo(f"Rotating {desired_rotation_degrees} degrees (target wheel ticks: {target_wheel_ticks}).")

        direction = 1 if desired_rotation_degrees > 0 else -1  # 1 for counter-clockwise, -1 for clockwise
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = direction * angular_speed

        while not rospy.is_shutdown():
            current_left_ticks = self.left_ticks - self.initial_left_ticks
            current_right_ticks = self.right_ticks - self.initial_right_ticks
            wheel_distance_travelled = (self.ticks_to_meters(abs(current_left_ticks)) + self.ticks_to_meters(abs(current_right_ticks))) / 2.0
            current_rotation_radians = (wheel_distance_travelled * 2) / self.wheel_separation
            current_rotation_degrees = self.degrees_to_radians(current_rotation_radians) * 180 / 3.14159


            if abs(current_rotation_degrees) >= abs(desired_rotation_degrees):
                self.stop_robot()
                rospy.loginfo(f"Reached desired rotation of {current_rotation_degrees:.2f} degrees.")
                break

            self.cmd_msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.cmd_msg)
            rospy.sleep(0.01) # Small delay for the loop

        return current_left_ticks, current_right_ticks

    # Robot drives in a square and then stops
    def move_robot(self):
        side_length = 1.0  # meters
        angle = 90.0  # degrees
        linear_speed = 0.2  # m/s
        angular_speed = 0.5  # rad/s

        rospy.loginfo("Drawing a square with closed-loop control...")

        # Move forward
        rospy.loginfo("Moving forward...")
        left_ticks, right_ticks = self.move_straight_closed_loop(side_length, linear_speed)
        rospy.loginfo(f"Forward movement completed. Left ticks: {left_ticks}, Right ticks: {right_ticks}")
        rospy.sleep(1)

        # Rotate 90 degrees
        rospy.loginfo("Rotating...")
        left_ticks, right_ticks = self.rotate_in_place_closed_loop(angle, angular_speed)
        rospy.loginfo(f"Rotation completed. Left ticks: {left_ticks}, Right ticks: {right_ticks}")
        rospy.sleep(1)

        # Move forward
        rospy.loginfo("Moving forward...")
        left_ticks, right_ticks = self.move_straight_closed_loop(side_length, linear_speed)
        rospy.loginfo(f"Forward movement completed. Left ticks: {left_ticks}, Right ticks: {right_ticks}")
        rospy.sleep(1)

        # Rotate 90 degrees
        rospy.loginfo("Rotating...")
        left_ticks, right_ticks = self.rotate_in_place_closed_loop(angle, angular_speed)
        rospy.loginfo(f"Rotation completed. Left ticks: {left_ticks}, Right ticks: {right_ticks}")
        rospy.sleep(1)

        # Move forward
        rospy.loginfo("Moving forward...")
        left_ticks, right_ticks = self.move_straight_closed_loop(side_length, linear_speed)
        rospy.loginfo(f"Forward movement completed. Left ticks: {left_ticks}, Right ticks: {right_ticks}")
        rospy.sleep(1)

        # Rotate 90 degrees
        rospy.loginfo("Rotating...")
        left_ticks, right_ticks = self.rotate_in_place_closed_loop(angle, angular_speed)
        rospy.loginfo(f"Rotation completed. Left ticks: {left_ticks}, Right ticks: {right_ticks}")
        rospy.sleep(1)

        # Move forward (final side)
        rospy.loginfo("Moving forward...")
        left_ticks, right_ticks = self.move_straight_closed_loop(side_length, linear_speed)
        rospy.loginfo(f"Forward movement completed. Left ticks: {left_ticks}, Right ticks: {right_ticks}")
        rospy.sleep(1)

        # Rotate 90 degrees to return to the starting orientation
        rospy.loginfo("Rotating to starting orientation...")
        left_ticks, right_ticks = self.rotate_in_place_closed_loop(angle, angular_speed)
        rospy.loginfo(f"Rotation completed. Left ticks: {left_ticks}, Right ticks: {right_ticks}")
        rospy.sleep(1)

        rospy.loginfo("Square drawing complete.")
        self.stop_robot()

    # Spin forever but listen to message callbacks
    def run(self):
        rospy.spin() # keeps node from exiting until node has shutdown

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass
