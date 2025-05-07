#!/usr/bin/env python3

#Python Libs
import sys, time

#numpy
import numpy as np

#OpenCV
import cv2
from cv_bridge import CvBridge

#ROS Libraries
import rospy
import roslib

#ROS Message Types
from sensor_msgs.msg import CompressedImage

class Lane_Detector:
    def __init__(self):
        self.cv_bridge = CvBridge()

        #### REMEMBER TO CHANGE THE TOPIC NAME! #####
        # Get the correct topic name by running rostopic list in Terminal 3
        self.image_sub = rospy.Subscriber('/majdoor/camera_node/image/compressed', CompressedImage, self.image_callback, queue_size=1)
        # If the topic is not compressed, use sensor_msgs.msg.Image and cv_bridge.imgmsg_to_cv2
        # self.image_sub = rospy.Subscriber('/majdoor/camera_node/image', Image, self.image_callback, queue_size=1)
        ############################################

        rospy.init_node("my_lane_detector", anonymous=True) # Added anonymous=True to prevent naming conflicts

    def image_callback(self,msg):
        # rospy.loginfo("image_callback") # Commented out to reduce terminal spam

        # Convert to opencv image
        # Use "bgr8" for color images. If the topic is not compressed, msg type is sensor_msgs.msg.Image
        try:
            img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        #### IMAGE PROCESSING FOR WHITE FILTERED IMAGE ####

        # 1. Crop the input image
        # You need to determine the correct cropping coordinates [y_min:y_max, x_min:x_max]
        # based on your camera's view to isolate the road.
        height, width, _ = img.shape
        # Adjust these coordinates for accurate cropping:
        crop_img = img[int(height/2):, :] # Example: keeps the bottom half

        # 2. Convert the cropped image to HSV Color Space
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # 3. Apply color filtering for White pixels
        # Adjusted values to be slightly more inclusive for white.
        # TUNE THESE VALUES based on your bag file images!
        lower_white = np.array([0, 0, 180])  # Slightly lower Value bound
        upper_white = np.array([255, 50, 255]) # Slightly higher Saturation bound

        # Create a binary mask for white pixels
        white_mask = cv2.inRange(hsv_img, lower_white, upper_white)

        # 4. Convert the white mask back to RGB for demonstration (as required by the task)
        # This converts the single-channel binary mask into a 3-channel image for display.
        white_filtered_rgb = cv2.cvtColor(white_mask, cv2.COLOR_GRAY2BGR)


        #### END OF IMAGE PROCESSING ####

        # Show the white-filtered image in a window
        cv2.imshow('White Filtered Image', white_filtered_rgb)
        cv2.waitKey(1) # Keep this small delay

    def run(self):
        rospy.spin() # Spin forever but listen to message callbacks

if __name__ == "__main__":
    try:
        lane_detector_instance = Lane_Detector()
        rospy.loginfo("Lane Detector Node Started")
        lane_detector_instance.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Lane Detector Node Interrupted")
    finally:
        # Clean up OpenCV windows when the node is shut down
        cv2.destroyAllWindows()
