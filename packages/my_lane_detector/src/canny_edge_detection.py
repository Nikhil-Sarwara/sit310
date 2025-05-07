import cv2
import os

# --- Configuration ---
# Set the filename of the image.
# Assumes the script and the image are in the same directory.
image_filename = 'Raw Camera Image_screenshot_07.05.2025.png' # **Ensure this matches your filename exactly**
image_path = image_filename # The path is just the filename if they are in the same directory

# Define your Canny thresholds here
low_threshold = 50   # **UPDATE THIS for each experiment**
high_threshold = 150 # **UPDATE THIS for each experiment**

# Define a name for the output screenshot (optional - you can save manually from the window)
# This will save the output in the same directory as the script
output_filename = f'canny_output_low{low_threshold}_high{high_threshold}.png'
output_path = output_filename
# ----------------------

# Check if the image file exists in the current directory
if not os.path.exists(image_path):
    print(f"Error: Image file not found at {image_path}")
    print(f"Please make sure '{image_filename}' is in the same directory as the script.")
else:
    # Load the image in grayscale
    # Canny typically works on grayscale images
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    if img is None:
        print(f"Error: Could not load image from {image_path}")
        print("Please check if the file is a valid image.")
    else:
        print(f"Successfully loaded image: {image_path}")

        # Apply Canny Edge Detection
        edges = cv2.Canny(img, low_threshold, high_threshold)

        # Display the original and edge-detected images
        cv2.imshow("Original Image (Grayscale)", img)
        cv2.imshow(f"Canny Edges (Low: {low_threshold}, High: {high_threshold})", edges)

        # Optional: Save the output image
        # cv2.imwrite(output_path, edges)
        # print(f"Canny output saved to {output_path}")

        print("\nDisplaying images. Press any key on one of the image windows to close them.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
