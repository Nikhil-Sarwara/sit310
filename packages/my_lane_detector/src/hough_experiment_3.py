import cv2
import numpy as np
import os
import sys

# --- Configuration ---
# Set the filename of the original image
original_image_filename = 'Raw Camera Image_screenshot_07.05.2025.png' # Ensure this matches your filename
original_image_path = original_image_filename

# Set the filename of the Canny edge-detected image to use as input for Hough
# **UPDATE THIS to the filename of one of your Canny output images**
canny_image_filename = 'canny_output_low75_high150.png' # Example - CHANGE THIS to your chosen Canny image filename
canny_image_path = canny_image_filename

# Define the parameter to vary and its value for this script
parameter_changed = "threshold"
hough_threshold_value = 200 # Third threshold value for Hough experiment

# Other HoughLinesP parameters (keep these constant for this part)
rho = 1           # Distance resolution of the accumulator in pixels.
theta = np.pi/180 # Angle resolution of the accumulator in radians.
minLineLength = 30 # Minimum line length. Line segments shorter than this are rejected.
maxLineGap = 10    # Maximum allowed gap between line segments to treat them as a single line.
# ----------------------

def run_hough_transform(original_img_path, canny_img_path, hough_threshold, rho, theta, minLineLength, maxLineGap, param_name, param_value):
    """Loads images, applies Hough Transform, draws lines, and displays."""

    # Load the original color image to draw lines on
    original_img_color = cv2.imread(original_img_path)
    if original_img_color is None:
        print(f"Error: Could not load original image from {original_img_path}")
        return

    # Load the Canny edge-detected image (should be grayscale)
    canny_img = cv2.imread(canny_image_path, cv2.IMREAD_GRAYSCALE)
    if canny_img is None:
        print(f"Error: Could not load Canny image from {canny_image_path}")
        return
    # Ensure Canny image is grayscale
    if len(canny_img.shape) > 2:
         print(f"Warning: Canny image is not grayscale. Converting.")
         canny_img = cv2.cvtColor(canny_img, cv2.COLOR_BGR2GRAY)


    print(f"Successfully loaded images.")
    print(f"Applying Hough Transform with {param_name}={param_value}")

    # Apply Probabilistic Hough Line Transform
    # It returns an array of lines in the format [x1, y1, x2, y2]
    lines = cv2.HoughLinesP(canny_img, rho, theta, hough_threshold,
                            minLineLength=minLineLength, maxLineGap=maxLineGap)

    # Create a copy of the original image to draw lines on
    line_img = np.copy(original_img_color) * 0 # Create a black image of the same size

    # Draw the detected lines on the blank image
    if lines is not None:
        print(f"Detected {len(lines)} lines.")
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # Draw line in green with thickness 2
            cv2.line(line_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    else:
        print("No lines detected with the current parameters.")

    # Combine the original image and the line image
    # Add the line image to the original image. Pixels in line_img are non-zero where lines are drawn.
    combined_img = cv2.addWeighted(original_img_color, 0.8, line_img, 1, 0)


    print(f"\nDisplaying images. Press Ctrl+C in the terminal to close the windows.")

    try:
        while True:
            # Display the combined image
            cv2.imshow(f"Hough Lines ({param_name}={param_value})", combined_img)

            # Wait for 1 millisecond. Necessary for window updates.
            if cv2.waitKey(1) & 0xFF == ord('q'): # Optional: close by pressing 'q' as well
                break

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Closing windows.")

    finally:
        cv2.destroyAllWindows()
        print("Windows closed.")


# --- Script Execution ---
if __name__ == '__main__':
    # Check if original image exists
    if not os.path.exists(original_image_path):
        print(f"Error: Original image file not found at {original_image_path}")
        print(f"Please make sure '{original_image_filename}' is in the same directory as the script.")
        sys.exit(1)

    # Check if Canny image exists
    if not os.path.exists(canny_image_path):
        print(f"Error: Canny image file not found at {canny_image_path}")
        print(f"Please make sure '{canny_image_filename}' is in the same directory as the script.")
        print("Ensure you have generated this Canny output image from the previous step.")
        sys.exit(1)

    # Run the Hough Transform function
    run_hough_transform(original_image_path, canny_image_path, hough_threshold_value, rho, theta, minLineLength, maxLineGap, parameter_changed, hough_threshold_value)

