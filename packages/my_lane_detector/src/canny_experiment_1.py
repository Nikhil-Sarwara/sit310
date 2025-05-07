import cv2
import os
import sys

# --- Configuration ---
image_filename = 'Raw Camera Image_screenshot_07.05.2025.png' # Ensure this matches your filename
image_path = image_filename

# Define your Canny thresholds for Experiment 1
low_threshold = 30
high_threshold = 90 # Example: A 1:3 ratio

# ----------------------

# Check if the image file exists
if not os.path.exists(image_path):
    print(f"Error: Image file not found at {image_path}")
    print(f"Please make sure '{image_filename}' is in the same directory as the script.")
    sys.exit(1)

img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

if img is None:
    print(f"Error: Could not load image from {image_path}")
    print("Please check if the file is a valid image.")
    sys.exit(1)

print(f"Successfully loaded image: {image_path}")
print(f"Applying Canny with Low Threshold: {low_threshold}, High Threshold: {high_threshold}")

# Apply Canny Edge Detection
edges = cv2.Canny(img, low_threshold, high_threshold)

print("\nDisplaying images. Press Ctrl+C in the terminal to close the windows.")

try:
    while True:
        # Display the original and edge-detected images
        cv2.imshow("Original Image (Grayscale)", img)
        cv2.imshow(f"Canny Edges (Exp 1: Low={low_threshold}, High={high_threshold})", edges)

        # Wait for 1 millisecond. This is necessary for the windows to update.
        # If a key is pressed, waitKey returns its ASCII value, otherwise -1.
        # We are not checking for specific key presses here, just keeping the loop alive.
        if cv2.waitKey(1) & 0xFF == ord('q'): # Optional: close by pressing 'q' as well
            break

except KeyboardInterrupt:
    # This block is executed when Ctrl+C is pressed
    print("\nCtrl+C detected. Closing windows.")

finally:
    # Ensure all windows are closed properly
    cv2.destroyAllWindows()
    print("Windows closed.")
