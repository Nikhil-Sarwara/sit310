#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray
# No need for time module for this task.

class Target_Follower:
    def __init__(self):

        #Initialize ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)

        ###### Init Pub/Subs. REMEMBER TO REPLACE "akandb" WITH YOUR ROBOT'S NAME #####
        # Make sure to replace 'akandb' below with your actual robot's name
        self.cmd_vel_pub = rospy.Publisher('/majdoor/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.tag_subscriber = rospy.Subscriber('/majdoor/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        ################################################################

        # --- Parameters for 'Move Towards Object' Behavior ---
        # Rotational Control (Keep object centered horizontally)
        # Dead zone for horizontal error (x) where the robot stops rotating
        self.dead_zone_angular = 0.05 # Adjust based on experimentation

        # Proportional gain for rotation when outside dead zone
        self.Kp_angular = 1.0 # Adjust this value

        # Linear Control (Maintain target distance)
        # Desired distance from the tag in meters when outside stop distance
        self.target_distance = 0.4 # Adjust this value based on desired following distance

        # Dead zone for distance error (z) where the robot stops moving linearly when outside stop distance
        self.dead_zone_linear = 0.05 # Adjust based on experimentation

        # Proportional gain for linear movement when outside stop distance and linear dead zone
        self.Kp_linear = 1.5 # Adjust this value

        # --- Stop Condition Parameter ---
        # Minimum distance to the object (z) before the robot stops completely
        self.stop_distance = 0.2 # Stop all movement if z is less than this value
        # --------------------------

        # Keep the node running and listening for messages
        rospy.spin()

    # Apriltag Detection Callback
    def tag_callback(self, msg):
        """
        Callback function for processing AprilTagDetectionArray messages.
        Calls the move_robot function with the list of detections.
        """
        self.move_robot(msg.detections)

    # Stop Robot before node has shut down.
    def clean_shutdown(self):
        """
        Function called on ROS node shutdown.
        Ensures the robot is stopped before the node exits.
        """
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Sends zero velocity to stop the robot
    def stop_robot(self):
        """
        Publishes a Twist2DStamped message with zero linear and angular velocity
        to stop the robot.
        """
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def move_robot(self, detections):
        """
        Controls the robot's movement based on April Tag detections.
        Implements stationary behavior when no tags, stop at distance, and 'Move Towards Object'.
        """

        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()

        if len(detections) == 0:
            # No tags detected - Stay stationary as requested
            cmd_msg.v = 0.0
            cmd_msg.omega = 0.0
            rospy.loginfo("No tags detected, staying stationary.")

        else:
            # Tags detected

            # Assuming x is the horizontal deviation and z is the distance
            # You confirmed x for horizontal, and z is typically distance in camera frame.
            x = detections[0].transform.translation.x
            z = detections[0].transform.translation.z # Assuming z is the distance to the tag

            # --- Stop Condition: Stop if too close to the object ---
            if z < self.stop_distance:
                cmd_msg.v = 0.0
                cmd_msg.omega = 0.0
                rospy.loginfo("Object is too close (z=%.4f < %.4f), stopping.", z, self.stop_distance)

            else:
                # --- If not too close, perform 'Move Towards Object' ---

                # --- Rotational Control (Keep object centered horizontally) ---
                # Calculate angular velocity based on horizontal error (x)
                if abs(x) < self.dead_zone_angular:
                    # If within the angular dead zone, stop rotating
                    cmd_msg.omega = 0.0
                    # rospy.loginfo("Object is horizontally centered.") # Optional: uncomment for debugging
                else:
                    # If outside the dead zone, calculate proportional velocity
                    # Standard proportional control: omega proportional to -x
                    # (assuming positive x is right, positive omega is left turn)
                    omega_proportional = -self.Kp_angular * x

                    # *** Apply the specific angular velocity logic: add 5.0 to magnitude ***
                    # If proportional velocity is non-negative, add 5.0
                    if omega_proportional >= 0:
                        cmd_msg.omega = omega_proportional + 5.0
                        # rospy.loginfo("Horizontal error=%.4f, Angular vel=%.4f (proportional %.4f + 5.0)", x, cmd_msg.omega, omega_proportional) # Optional: uncomment
                    # If proportional velocity is negative, subtract 5.0
                    else: # omega_proportional is negative
                        cmd_msg.omega = omega_proportional - 5.0
                        # rospy.loginfo("Horizontal error=%.4f, Angular vel=%.4f (proportional %.4f - 5.0)", x, cmd_msg.omega, omega_proportional) # Optional: uncomment
                    # ***************************************************************************


                # --- Linear Control (Maintain target distance) ---
                # Calculate distance error
                distance_error = z - self.target_distance

                # Calculate linear velocity based on distance error (z)
                if abs(distance_error) < self.dead_zone_linear:
                    # If within the linear dead zone, stop moving linearly
                    cmd_msg.v = 0.0
                    # rospy.loginfo("Object is at target distance.") # Optional: uncomment for debugging
                else:
                    # If outside the dead zone, use Proportional control for linear movement
                    # Move forward if too far (distance_error > 0), backward if too close (distance_error < 0)
                    # Use -Kp_linear to move towards the target distance
                    cmd_msg.v = -self.Kp_linear * distance_error
                    # rospy.loginfo("Distance error=%.4f, moving linearly...", distance_error) # Optional: uncomment for debugging

                # Log the current state and velocities when moving/rotating (and not too close)
                rospy.loginfo("Tag detected at x=%.4f, z=%.4f. Angular vel=%.4f, Linear vel=%.4f",
                              x, z, cmd_msg.omega, cmd_msg.v)


        # Publish the calculated velocity command
        self.cmd_vel_pub.publish(cmd_msg)


if __name__ == '__main__':
    try:
        # Create an instance of the Target_Follower class
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        # Handle ROS interrupt exceptions
        pass

