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


class Yellow_Lane_Detector:
    def __init__(self):
        self.cv_bridge = CvBridge()

        #### IMPORTANT: REPLACE WITH YOUR ACTUAL TOPIC NAME! #####
        # Get the correct topic name by running rostopic list in Terminal 3
        # Use the appropriate message type (CompressedImage or Image)
        self.image_sub = rospy.Subscriber('/majdoor/camera_node/image/compressed', CompressedImage, self.image_callback, queue_size=1)
        # If the topic is not compressed, use the line below instead:
        # self.image_sub = rospy.Subscriber('/majdoor/camera_node/image', Image, self.image_callback, queue_size=1)
        ##########################################################

        rospy.init_node("yellow_filter_node", anonymous=True) # Added anonymous=True

    def image_callback(self, msg):
        # rospy.loginfo("image_callback") # Commented out to reduce terminal spam

        # Convert to opencv image
        try:
            # Adjust encoding ("bgr8" or "rgb8") based on your camera/topic
            # Use self.cv_bridge.imgmsg_to_cv2(msg, "bgr8") for sensor_msgs.msg.Image
            img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        #### IMAGE PROCESSING FOR YELLOW FILTERED IMAGE (IN YELLOW COLOR) ####

        # 1. Crop the input image
        # Use the same cropping coordinates as your white filter node.
        height, width, _ = img.shape
        # Adjust these coordinates for accurate cropping (should be the same as white filter):
        crop_img = img[int(height/2):, :] # Example: keeps the bottom half

        # 2. Convert the cropped image to HSV Color Space
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # 3. Apply color filtering for Yellow pixels
        # Use the tuned values from your previous step.
        lower_yellow = np.array([15, 80, 80])  # Example tuned value
        upper_yellow = np.array([45, 255, 255]) # Example tuned value

        # Create a binary mask for yellow pixels
        yellow_mask = cv2.inRange(hsv_img, lower_yellow, upper_yellow)

        # 4. Create a 3-channel image to display the filtered area in yellow
        # Create a black image of the same size as the mask
        yellow_filtered_color = np.zeros_like(crop_img)

        # Set pixels in the yellow_filtered_color image to yellow where the yellow_mask is white (255)
        # In BGR, yellow is (0, 255, 255)
        yellow_filtered_color[yellow_mask > 0] = (0, 255, 255)


        #### END OF IMAGE PROCESSING ####

        # Show the yellow-filtered image in yellow color in a window
        cv2.imshow('Yellow Filtered Image', yellow_filtered_color)
        cv2.waitKey(1) # Small delay to update the window

    def run(self):
        rospy.spin() # Spin forever, listening to message callbacks

if __name__ == "__main__":
    try:
        yellow_detector_instance = Yellow_Lane_Detector()
        rospy.loginfo("Yellow Filter Node Started")
        yellow_detector_instance.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Yellow Filter Node Interrupted")
    finally:
        # Clean up OpenCV windows
        cv2.destroyAllWindows()
