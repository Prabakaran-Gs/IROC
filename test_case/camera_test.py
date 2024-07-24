import cv2

# Open a connection to the camera (0 is the default camera)
cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # If frame reading was successful
    if ret:
        # Display the resulting frame
        cv2.imshow('Camera Feed', frame)
    else:
        print("Error: Could not read frame.")
        break

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.imwrite("arm_position.jpg",frame)
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
