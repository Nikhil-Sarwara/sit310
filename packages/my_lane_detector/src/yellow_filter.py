#!/usr/bin/env python3
import sys
import time
import os
import numpy as np
import cv2

image_filename = 'Raw Camera Image_screenshot_07.05.2025.png'
image_path = image_filename

lower_yellow_hsv = np.array([15, 80, 80])
upper_yellow_hsv = np.array([45, 255, 255])

def process_single_image(image_path, lower_hsv, upper_hsv):
    img = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if img is None:
        print(f"Error: Could not load image from {image_path}")
        return None, None
    print(f"Successfully loaded image: {image_path}")
    height, width, _ = img.shape
    crop_img = img[int(height/2):, :]
    hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
    yellow_mask = cv2.inRange(hsv_img, lower_hsv, upper_hsv)
    yellow_filtered_color = np.zeros_like(crop_img)
    yellow_filtered_color[yellow_mask > 0] = (0, 255, 255)
    return yellow_filtered_color, yellow_mask

if __name__ == "__main__":
    if not os.path.exists(image_path):
        print(f"Error: Image file not found at {image_path}")
        sys.exit(1)

    filtered_image, binary_mask = process_single_image(image_path, lower_yellow_hsv, upper_yellow_hsv)

    if filtered_image is not None:
        original_cropped = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if original_cropped is not None:
             original_cropped = original_cropped[int(original_cropped.shape[0]/2):, :]
        else:
             print("Warning: Could not reload original image for display.")

        try:
            while True:
                if original_cropped is not None:
                    cv2.imshow('Original Image (Cropped)', original_cropped)
                cv2.imshow('Yellow Filtered Color', filtered_image)
                cv2.imshow('Yellow Binary Mask', binary_mask)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break

        except KeyboardInterrupt:
            pass

        finally:
            cv2.destroyAllWindows()
            print("Windows closed.")

    else:
        print("Image processing failed.")
