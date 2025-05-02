

import cv2
# import time

def main():
    # Initial setup
    capture = cv2.VideoCapture(0)  # Open the default camera

    while True:
        # Get an image from the camera
        ret, image = capture.read()

        if not ret:
            print("Failed to capture image")
            capture.release()
            cv2.destroyAllWindows()
            return

        # Show acquired image
        cv2.imshow(window_name, image)

        # Update the frame count
        frame_count += 1

    # Cleanup
    capture.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
