#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray
import time # Import the time module (though rospy.Time is used for ROS time)

class Target_Follower:
    def __init__(self):

        #Initialize ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)

        ###### Init Pub/Subs. REMEMBER TO REPLACE "akandb" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher('/majdoor/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/majdoor/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        ################################################################

        # --- Class Attributes ---
        self.last_detection_time = rospy.Time.now() # Initialize last detection time
        self.seeking_omega = 5.0 # Angular velocity for seeking

        # --- Parameters for Looking ---
        self.dead_zone_angular = 0.05 # Adjust based on experimentation for centering
        self.Kp_angular = 1.0         # Proportional gain for rotation (Adjust this if needed)
        # --------------------------


        rospy.spin() # Spin forever but listen to message callbacks

    # Apriltag Detection Callback
    def tag_callback(self, msg):
        # This callback is called whenever an AprilTagDetectionArray message is received.
        # We will handle the robot movement logic here or call another function.
        self.move_robot(msg.detections)

    # Stop Robot before node has shut down.
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Sends zero velocity to stop the robot
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def move_robot(self, detections):

        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0 # Linear velocity is always 0 for these features

        if len(detections) == 0:
            # No tags detected
            time_since_last_detection = (rospy.Time.now() - self.last_detection_time).to_sec()

            if time_since_last_detection > 2.0:
                # If no tags detected for more than 2 seconds, start seeking
                cmd_msg.omega = self.seeking_omega
                rospy.loginfo("No tags detected for > 2s, seeking object...")
            else:
                # If no tags detected but less than 2 seconds have passed since last seen, stop
                cmd_msg.omega = 0.0
                rospy.loginfo("No tags detected, waiting...")

        else:
            # Tags detected
            # Update the last detection time
            self.last_detection_time = rospy.Time.now()

            # Assuming x is the horizontal deviation from the center
            x = detections[0].transform.translation.x

            # --- Rotational Control (Look at the Object) ---
            if abs(x) < self.dead_zone_angular:
                # If within the angular dead zone, stop rotating
                cmd_msg.omega = 0.0
                rospy.loginfo("Object is horizontally centered.")
            else:
                # If outside the dead zone, calculate proportional velocity
                omega_proportional = -self.Kp_angular * x # Standard proportional control

                # *** Apply the "add 5" logic based on the sign of the proportional output ***
                if omega_proportional >= 0:
                    cmd_msg.omega = omega_proportional + 5.0
                    rospy.loginfo("Horizontal error=%f, Angular vel=%.6f (proportional %.6f + 5.0)", x, cmd_msg.omega, omega_proportional)
                else: # omega_proportional is negative
                    cmd_msg.omega = omega_proportional - 5.0
                    rospy.loginfo("Horizontal error=%f, Angular vel=%.6f (proportional %.6f - 5.0)", x, cmd_msg.omega, omega_proportional)
                # ***************************************************************************

            # Linear velocity remains 0.0 as set at the beginning of the function


        self.cmd_vel_pub.publish(cmd_msg)


if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass
