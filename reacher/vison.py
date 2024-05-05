import cv2
import numpy as np

def main():
    # Start capturing video from the webcam
    cap = cv2.VideoCapture(0)

    while True:
        # Get video frame
        conn, frame = cap.read()

        # check if camera is connected
        if not conn:
            break

        # Chnage from rgb to hsv colorspace 
        # better for detecting a red hue rather than red color value
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the range of red color in HSV
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([5, 255, 255])
        lower_red_hues = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([175, 120, 70])
        upper_red = np.array([180, 255, 255])
        upper_red_hues = cv2.inRange(hsv, lower_red, upper_red)

        # Combine masks for red hues
        mask = upper_red_hues

        # filter the frame for only the masked red pixels
        output_hsv = frame
        output_hsv[np.where(mask==0)] = 0

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Process each contour
        for contour in contours:
            area = cv2.contourArea(contour)
            # Filter small areas to reduce noise
            if area > 100:
                # Calculate circularity to identify dots
                perimeter = cv2.arcLength(contour, True)
                if perimeter == 0:
                    continue
                circularity = 4 * np.pi * (area / (perimeter * perimeter))
                # Threshold for circularity can be adjusted, closer to 1 is perfect circle
                if circularity > 0.7:  # Adjust circularity threshold as needed
                    # Calculate bounding circle
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    radius = int(radius)
                    # Draw the circle
                    cv2.circle(output_hsv, center, radius, (0, 255, 0), 2)

        # Display the result
        cv2.imshow('Red Dot Tracker', output_hsv)

        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture object and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
