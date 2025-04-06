#!/usr/bin/env python3

# Author: Nikhil
# Student, Deakin University
# Unit: SIT310 Robotics Application Development
# Intake: [Specify your intake - e.g., Trimester 1 2025]
# This code is part of the 'glow' ROS package.
# My Duckiebot name is: majdoor

# Import the rospy library for ROS functionalities in Python
import rospy
# Import the Twist message type from the geometry_msgs package.
# Twist is used to send velocity commands to the turtle.
from geometry_msgs.msg import Twist
# Import the time module for introducing delays. While rospy.sleep() is generally preferred in ROS,
# time.sleep() is used here for simplicity in this introductory example.
import time

# Define a class named SquareTurtle to encapsulate the logic for drawing squares.
class SquareTurtle:
    # The constructor method of the class, called when a SquareTurtle object is created.
    def __init__(self):
        # Initialize the ROS node with a unique name 'square_turtle_node'.
        # The 'anonymous=True' flag ensures that if multiple nodes with the same name are run,
        # ROS will automatically append a unique ID to the name.
        rospy.init_node('square_turtle_node', anonymous=True)
        # Create a publisher object that will publish messages of type Twist to the '/turtle1/cmd_vel' topic.
        # This topic is used by the turtlesim node to receive velocity commands.
        # The 'queue_size=10' argument limits the number of outgoing messages that can be buffered
        # if the subscriber (turtlesim) is not receiving them quickly enough.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        # Create a Rate object to control the publishing frequency. Here, it's set to 10 Hz,
        # meaning we aim to loop at most 10 times per second. Note that we are using time.sleep()
        # in this example, which might not strictly adhere to this rate.
        self.rate = rospy.Rate(10)  # 10 Hz

    # Method to command the turtle to draw one side of the square.
    def draw_square_side(self, side_length, is_forward=True):
        # Create a Twist message object to hold the velocity commands.
        vel_msg = Twist()

        # Set the linear velocity to move forward or backward.
        if is_forward:
            vel_msg.linear.x = side_length  # Positive value for forward movement. Adjust for desired speed.
        else:
            vel_msg.linear.x = -side_length # Negative value for backward movement.

        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        # Publish the velocity message to the '/turtle1/cmd_vel' topic.
        self.velocity_publisher.publish(vel_msg)
        # Introduce a delay to allow the turtle to move for the specified duration.
        # The duration is roughly proportional to the side length (assuming a constant speed).
        time.sleep(1) # Adjust time based on desired speed and side length

    # Method to command the turtle to rotate by 90 degrees.
    def rotate(self, clockwise=True):
        # Create a Twist message object for rotation.
        vel_msg = Twist()
        # Set linear velocity to zero as we only want to rotate.
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        # Set the angular velocity for rotation around the Z-axis (yaw).
        # 90 degrees in radians is approximately 1.5708.
        if clockwise:
            vel_msg.angular.z = -1.5708  # Negative value for clockwise rotation.
        else:
            vel_msg.angular.z = 1.5708   # Positive value for counter-clockwise rotation.
        # Publish the rotation command.
        self.velocity_publisher.publish(vel_msg)
        # Introduce a delay for the rotation to complete.
        time.sleep(1) # Adjust time for rotation

    # The main method that contains the logic to make the turtle draw squares.
    def run(self):
        # Define the side length of the square. You can adjust this value.
        side_length = 2.0
        # Log an informational message to the ROS console.
        rospy.loginfo("Starting to draw squares...")
        # Enter an infinite loop that continues until the ROS node is shut down (e.g., by pressing Ctrl+C).
        while not rospy.is_shutdown():
            # Loop four times to draw the four sides of the square.
            for i in range(4):
                # Call the method to draw one side of the square.
                self.draw_square_side(side_length)
                # After drawing a side, rotate by 90 degrees clockwise.
                self.rotate(clockwise=True)

            # Optional: Add a small pause between drawing consecutive squares.
            rospy.sleep(rospy.Duration(0.5)) # Using rospy.sleep for better ROS integration

            # Check if the ROS node has been requested to shut down within the loop.
            if rospy.is_shutdown():
                break
            # Log a message indicating that a square has been completed and a new one is starting.
            rospy.loginfo("Finished drawing a square. Starting a new one.")

# The main entry point of the script. This block is executed when the script is run directly.
if __name__ == '__main__':
    try:
        # Create an instance of the SquareTurtle class.
        turtle_drawer = SquareTurtle()
        # Call the run method of the SquareTurtle object to start drawing squares.
        turtle_drawer.run()
    # Handle the ROSInterruptException, which is typically raised when the user interrupts the program (e.g., with Ctrl+C).
    except rospy.ROSInterruptException:
        pass
