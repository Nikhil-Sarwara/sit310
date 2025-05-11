import cv2
import numpy as np
import os
import sys

# --- Configuration ---
original_image_filename = 'Raw Camera Image_screenshot_07.05.2025.png'
original_image_path = original_image_filename

lower_yellow_hsv = np.array([20, 100, 100])
upper_yellow_hsv = np.array([40, 255, 255])

# Parameters for simulating lighting changes (Experiment with these)
alpha_bright = 1.5  # Factor for brightness/contrast (>1 for brighter)
beta_bright = 15    # Offset for brightness (positive for brighter)

alpha_dark = 0.6    # Factor for brightness/contrast (<1 for darker)
beta_dark = 0       # Offset for brightness (can be negative too, but alpha is usually enough)
# ----------------------

def apply_yellow_detection(image, lower_hsv, upper_hsv):
    """Converts image to HSV, applies color thresholding for yellow, returns the mask."""
    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    yellow_mask = cv2.inRange(hsv_img, lower_hsv, upper_hsv)
    return yellow_mask

def main():
    # Check if the original image file exists
    if not os.path.exists(original_image_path):
        print(f"Error: Original image file not found at {original_image_path}")
        sys.exit(1)

    # Load the original color image
    original_img = cv2.imread(original_image_path)
    if original_img is None:
        print(f"Error: Could not load original image from {original_image_path}")
        sys.exit(1)

    print(f"Successfully loaded image: {original_image_path}")

    # --- Simulate Different Lighting Conditions ---
    img_brighter = cv2.convertScaleAbs(original_img, alpha=alpha_bright, beta=beta_bright)
    img_darker = cv2.convertScaleAbs(original_img, alpha=alpha_dark, beta=beta_dark)

    cv2.imshow('Simulated Brighter Image', img_brighter)
    cv2.imshow('Simulated Darker Image', img_darker)


    # --- Apply Yellow Detection (Using Fixed BEST Parameters) to ALL Images ---

    print("\nApplying yellow detection with fixed parameters to all images...")

    # Detection on Original Image
    mask_original = apply_yellow_detection(original_img, lower_yellow_hsv, upper_yellow_hsv)
    result_original = cv2.bitwise_and(original_img, original_img, mask=mask_original)
    cv2.imshow('Yellow Detection (Original Light)', result_original) # Or just show mask_original

    # Detection on Brighter Simulated Image
    mask_brighter = apply_yellow_detection(img_brighter, lower_yellow_hsv, upper_yellow_hsv)
    result_brighter = cv2.bitwise_and(img_brighter, img_brighter, mask=mask_brighter)
    cv2.imshow('Yellow Detection (Brighter Light)', result_brighter) # Or just show mask_brighter


    # Detection on Darker Simulated Image
    mask_darker = apply_yellow_detection(img_darker, lower_yellow_hsv, upper_yellow_hsv)
    result_darker = cv2.bitwise_and(img_darker, img_darker, mask=mask_darker)
    cv2.imshow('Yellow Detection (Darker Light)', result_darker) # Or just show mask_darker


    print("\nDisplaying results. Press any key on one of the image windows to close them.")
    cv2.waitKey(0) # Wait indefinitely until a key is pressed
    cv2.destroyAllWindows()
    print("Windows closed.")

if __name__ == '__main__':
    main()
