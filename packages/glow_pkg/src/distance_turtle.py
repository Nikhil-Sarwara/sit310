#!/usr/bin/env python3

# My Name: Nikhil
# My University: Deakin University
# My Student Unit: SIT310 Robotics Application Development
# This is for Task 3.2C – ROS Turtlesim Distance

# Import Dependencies
import rospy
from geometry_msgs.msg import Twist # Although not directly used for distance calculation in this task, it's in the template
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import time
import math 

class DistanceReader:
    def __init__(self):

        # Initialize the node
        rospy.init_node('turtlesim_distance_node', anonymous=True)

        # Initialize subscriber: input the topic name, message type and callback signature
        rospy.Subscriber("/turtle1/pose", Pose,self.callback)

        # Initialize publisher: input the topic name, message type and msg queue size
        self.distance_publisher = rospy.Publisher('/turtle_dist', Float64, queue_size=10)

        # Printing to the terminal, ROS style
        rospy.loginfo("Initialized node!")

        # Store the previous position
        self.previous_x = None
        self.previous_y = None
        self.total_distance_travelled = 0.0

        # This blocking function call keeps python from exiting until node is stopped
        rospy.spin()

    # Whenever a message is received from the specified subscriber, this function will be called
    def callback(self,msg):
        rospy.loginfo("Turtle Position: x=%s, y=%s, Total Distance: %s", msg.x, msg.y, self.total_distance_travelled)

        ########## YOUR CODE GOES HERE ##########
        # Calculate the distance the turtle has travelled and publish it

        current_x = msg.x
        current_y = msg.y

        if self.previous_x is not None and self.previous_y is not None:
            delta_x = current_x - self.previous_x
            delta_y = current_y - self.previous_y
            distance_moved = math.sqrt(delta_x**2 + delta_y**2)
            self.total_distance_travelled += distance_moved
            self.publish_distance()
        else:
            # Initialize previous position on the first callback
            self.previous_x = current_x
            self.previous_y = current_y

        self.previous_x = current_x
        self.previous_y = current_y

        ###########################################

    def publish_distance(self):
        distance_message = Float64()
        distance_message.data = self.total_distance_travelled
        self.distance_publisher.publish(distance_message)

if __name__ == '__main__':

    try:
        distance_reader_class_instance = DistanceReader()
    except rospy.ROSInterruptException:
        pass
