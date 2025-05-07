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


class Raw_Image_Viewer:
    def __init__(self):
        self.cv_bridge = CvBridge()

        #### IMPORTANT: REPLACE WITH YOUR ACTUAL TOPIC NAME! #####
        # Get the correct topic name by running rostopic list in Terminal 3
        # Use the appropriate message type (CompressedImage or Image)
        self.image_sub = rospy.Subscriber('/majdoor/camera_node/image/compressed', CompressedImage, self.image_callback, queue_size=1)
        # If the topic is not compressed, use the line below instead:
        # self.image_sub = rospy.Subscriber('/majdoor/camera_node/image', Image, self.image_callback, queue_size=1)
        ##########################################################

        rospy.init_node("raw_image_viewer", anonymous=True) # Added anonymous=True

    def image_callback(self, msg):
        # Convert to opencv image
        try:
            # Adjust encoding ("bgr8" or "rgb8") based on your camera/topic
            # Use self.cv_bridge.imgmsg_to_cv2(msg, "bgr8") for sensor_msgs.msg.Image
            img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # Display the raw image
        cv2.imshow('Raw Camera Image', img)
        cv2.waitKey(1) # Small delay to update the window

    def run(self):
        rospy.spin() # Spin forever, listening to message callbacks

if __name__ == "__main__":
    try:
        raw_viewer_instance = Raw_Image_Viewer()
        rospy.loginfo("Raw Image Viewer Node Started")
        raw_viewer_instance.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Raw Image Viewer Node Interrupted")
    finally:
        # Clean up OpenCV windows
        cv2.destroyAllWindows()
