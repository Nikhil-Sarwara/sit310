#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray

class Autopilot:
    def __init__(self):
        
        #Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)

        self.robot_state = "LANE_FOLLOWING" # Default state is LANE_FOLLOWING
        self.ignore_tags = False # Flag for high-credit task: temporarily ignore tags after a stop

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        self.cmd_vel_pub = rospy.Publisher('/majdoor/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/majdoor/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/majdoor/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)

        rospy.spin() # Spin forever but listen to message callbacks

    # Apriltag Detection Callback
    def tag_callback(self, msg):
        # Only process AprilTag detections if the robot is in LANE_FOLLOWING state
        if self.robot_state != "LANE_FOLLOWING":
            return
        
        # If we are currently ignoring tags (e.g., after a recent stop sign), return early
        if self.ignore_tags:
            return

        self.move_robot(msg.detections)
 
    # Stop Robot before node has shut down. This ensures the robot keep moving with the latest velocity command
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

    def set_state(self, state):
        self.robot_state = state
        state_msg = FSMState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.state = self.robot_state
        self.state_pub.publish(state_msg)

    def enable_tag_detection(self, event):
        """Callback to re-enable AprilTag detection after a delay."""
        self.ignore_tags = False
        rospy.loginfo("AprilTag detection re-enabled.")

    def move_robot(self, detections):

        if len(detections) == 0:
            return

        for detection in detections:
            # We are interested in a specific tag, now with tag_id = 20
            if detection.tag_id == 20: 
                # The 'z' coordinate represents the distance to the tag from the camera
                z_distance = detection.transform.translation.z
                rospy.loginfo("Tag (ID: %d) detected at distance: %f meters", detection.tag_id, z_distance)

                # Define the target stopping distance (0.25 meters)
                STOP_DISTANCE = 0.25 # meters

                if z_distance <= STOP_DISTANCE:
                    rospy.loginfo("Tag is within %f meters. Stopping robot.", STOP_DISTANCE)
                    self.set_state("NORMAL_JOYSTICK_CONTROL") # Stop Lane Following by overriding the state
                    self.stop_robot() # Stop the robot
                    rospy.loginfo("Robot stopped. Waiting for a fixed period (3 seconds)...")
                    rospy.sleep(3.0) # Robot stops and waits for 3 seconds

                    # Implement high-credit behavior: temporarily ignore tags
                    self.ignore_tags = True 
                    # Set up a timer to re-enable tag detection after a delay (5 seconds)
                    rospy.Timer(rospy.Duration(5.0), self.enable_tag_detection, oneshot=True) 
                    rospy.loginfo("Ignoring tags for 5 seconds to prevent immediate re-detection.")

                    # Move forward slightly to ensure the tag is out of view
                    rospy.loginfo("Moving forward slightly to clear the tag...")
                    cmd_msg = Twist2DStamped()
                    cmd_msg.header.stamp = rospy.Time.now()
                    cmd_msg.v = 0.2 # Small forward velocity
                    cmd_msg.omega = 0.0
                    self.cmd_vel_pub.publish(cmd_msg)
                    rospy.sleep(1.0) # Move forward for 1 second
                    self.stop_robot() # Stop again after moving
                    rospy.loginfo("Robot stopped after moving forward.")


                    rospy.loginfo("Resuming Lane Following...")
                    self.set_state("LANE_FOLLOWING") # Go back to lane following
                    break # Process only one tag at a time if multiple are detected
                else:
                    # If the tag is detected but not within the stopping distance,
                    # the robot should continue lane following.
                    rospy.loginfo("Tag detected at %f meters, continuing lane following.", z_distance)

if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass
