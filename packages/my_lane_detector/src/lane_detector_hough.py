#!/usr/bin/env python3

#Python Libs
import sys
import time

#numpy
import numpy as np

#OpenCV
import cv2
from cv_bridge import CvBridge

#ROS Libraries
import rospy

#ROS Message Types
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image # Import Image message type as well, just in case


class Lane_Detector_Hough:
    def __init__(self):
        self.cv_bridge = CvBridge()

        #### IMPORTANT: REPLACE WITH YOUR ACTUAL TOPIC NAME! #####
        # Get the correct topic name by running rostopic list in Terminal 3
        # Use the appropriate message type (CompressedImage or Image)
        self.image_sub = rospy.Subscriber('/majdoor/camera_node/image/compressed', CompressedImage, self.image_callback, queue_size=1)
        # If the topic is not compressed, use the line below instead:
        # self.image_sub = rospy.Subscriber('/majdoor/camera_node/image', Image, self.image_callback, queue_size=1)
        ##########################################################

        rospy.init_node("lane_detector_hough_node", anonymous=True) # Added anonymous=True

    def image_callback(self, msg):
        # Convert to opencv image
        try:
            # Adjust encoding ("bgr8" or "rgb8") based on your camera/topic
            # Use self.cv_bridge.imgmsg_to_cv2(msg, "bgr8") for sensor_msgs.msg.Image
            img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        #### IMAGE PROCESSING FOR HOUGH TRANSFORM AND LINE DRAWING ####

        # 1. Crop the input image
        # Use the same cropping coordinates as your previous nodes.
        height, width, _ = img.shape
        # Adjust these coordinates for accurate cropping:
        crop_img = img[int(height/2):, :] # Example: keeps the bottom half

        # Create a copy of the cropped image to draw lines on
        img_with_lines = np.copy(crop_img)

        # 2. Convert the cropped image to HSV Color Space
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # 3. Apply color filtering for White pixels (to create a mask for Hough)
        # Use the tuned values from your first video submission.
        # These values are kept the same as the previous white tuning example.
        lower_white = np.array([0, 0, 180])  # Example tuned value
        upper_white = np.array([255, 50, 255]) # Example tuned value
        white_mask = cv2.inRange(hsv_img, lower_white, upper_white)

        # 4. Apply color filtering for Yellow pixels (to create a mask for Hough)
        # Adjusted values to be more inclusive for yellow.
        # TUNE THESE VALUES further based on your bag file images!
        # Expanded Hue range, lower min Saturation and Value
        lower_yellow = np.array([15, 80, 80])  # Lowered Hue, Saturation, and Value minimums
        upper_yellow = np.array([45, 255, 255]) # Widened Hue range, kept max Sat/Value

        # Create a binary mask for yellow pixels
        yellow_mask = cv2.inRange(hsv_img, lower_yellow, upper_yellow)

        # 5. Apply Canny Edge Detector to the cropped image
        # Tune these thresholds to detect prominent edges, including lane lines.
        canny_output = cv2.Canny(crop_img, 50, 150) # Example thresholds - TUNE THIS!

        # 6. Apply Hough Transform to the White-filtered image (mask)
        # Corrected 'maxGap' to 'maxLineGap'
        # Tune these parameters (rho, theta, threshold, minLineLength, maxLineGap)
        white_lines = cv2.HoughLinesP(white_mask, 1, np.pi/180, 50, minLineLength=50, maxLineGap=5) # Example parameters - TUNE THIS!

        # 7. Apply Hough Transform to the Yellow-filtered image (mask)
        # Corrected 'maxGap' to 'maxLineGap'
        # Tune these parameters similarly to the white lines.
        yellow_lines = cv2.HoughLinesP(yellow_mask, 1, np.pi/180, 50, minLineLength=50, maxLineGap=5) # Example parameters - TUNE THIS!

       # 8. Draw lines found on both Hough Transforms on the cropped image
        if white_lines is not None:
            for line in white_lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(img_with_lines, (x1, y1), (x2, y2), (0, 0, 255), 2) # Draw white lines in RED (BGR)

        if yellow_lines is not None:
            for line in yellow_lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(img_with_lines, (x1, y1), (x2, y2), (0, 255, 255), 2) # Draw yellow lines in YELLOW (BGR)

        #### END OF IMAGE PROCESSING ####

        # Show the image with detected lines in a window
        cv2.imshow('Lines Detected on Cropped Image', img_with_lines)
        cv2.waitKey(1) # Small delay to update the window

    def run(self):
        rospy.spin() # Spin forever, listening to message callbacks

if __name__ == "__main__":
    try:
        lane_detector_hough_instance = Lane_Detector_Hough()
        rospy.loginfo("Lane Detector with Hough Node Started")
        lane_detector_hough_instance.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Lane Detector with Hough Node Interrupted")
    finally:
        # Clean up OpenCV windows
        cv2.destroyAllWindows()
