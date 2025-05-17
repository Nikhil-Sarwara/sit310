import cv2
import numpy as np
import os
import sys
import time

lower_yellow_rgb = np.array([0, 100, 100])
upper_yellow_rgb = np.array([100, 255, 255])

def process_rgb_for_yellow(image_segment, lower_rgb, upper_rgb):
    if image_segment is None:
        print("Error: Input image segment is None for RGB processing.")
        return None, None

    yellow_mask_rgb = cv2.inRange(image_segment, lower_rgb, upper_rgb)

    yellow_filtered_color_rgb = np.zeros_like(image_segment)
    yellow_filtered_color_rgb[yellow_mask_rgb > 0] = (0, 255, 255)

    return yellow_filtered_color_rgb, yellow_mask_rgb

if __name__ == "__main__":
    image_filename = 'Raw Camera Image_screenshot_07.05.2025.png'
    image_path = image_filename

    lower_yellow_hsv = np.array([15, 80, 80])
    upper_yellow_hsv = np.array([45, 255, 255])

    original_image = cv2.imread(image_path, cv2.IMREAD_COLOR)

    if original_image is None:
        print(f"Error: Could not load image from {image_path}")
        sys.exit(1)
    print(f"Successfully loaded image: {image_path}")

    height, width, _ = original_image.shape
    cropped_image = original_image[int(height/2):, :]

    hsv_img = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)
    yellow_mask_hsv = cv2.inRange(hsv_img, lower_yellow_hsv, upper_yellow_hsv)
    yellow_filtered_color_hsv = np.zeros_like(cropped_image)
    yellow_filtered_color_hsv[yellow_mask_hsv > 0] = (0, 255, 255)

    yellow_filtered_color_rgb, yellow_mask_rgb = process_rgb_for_yellow(
        cropped_image, lower_yellow_rgb, upper_yellow_rgb
    )

    try:
        while True:
            cv2.imshow('Original Image (Cropped)', cropped_image)

            cv2.imshow('Yellow Filtered Color (HSV)', yellow_filtered_color_hsv)
            cv2.imshow('Yellow Binary Mask (HSV)', yellow_mask_hsv)

            if yellow_filtered_color_rgb is not None:
                 cv2.imshow('Yellow Filtered Color (RGB)', yellow_filtered_color_rgb)
                 cv2.imshow('Yellow Binary Mask (RGB)', yellow_mask_rgb)


            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    except KeyboardInterrupt:
        pass

    finally:
        cv2.destroyAllWindows()
        print("Windows closed.")
