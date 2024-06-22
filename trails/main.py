import cv2
import time
import numpy as np

def draw_poly(self,image,polygon_coords):
          #logger.log_info("Drawing polygon")
        #polygon_coords = [(301, 1080), (591, 532), (1333, 511) , (1649, 1080)]

          # Convert the polygon coordinates to a NumPy array
        polygon_pts = np.array(polygon_coords, np.int32)
        polygon_pts = polygon_pts.reshape((-1, 1, 2))

          # Draw the polygon on the image
        cv2.polylines(image, [polygon_pts], isClosed=True, color=(0, 255, 0), thickness=3)

        return image



cap = cv2.VideoCapture(0)

# Initialize variables
frame_count = 0

ctr = 0
# Loop through the video frames
while cap.isOpened():
    # Read a frame from the video
    success, frame = cap.read()

    if success:
        frame_count += 1
        start_time = time.time()

        # frame = dr

        # Run YOLOv8 inference on the frame
        cv2.imshow("Output ",frame)
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            #break
            cv2.imwrite(f"sample{ctr}.jpg",frame)
            ctr += 1
            print(ctr)
            frame_count = 0
        
    else:
        # Break the loop if the end of the video is reached
        break

# Release the video capture object and close the display window
cap.release()
# cv2.destroyAllWindows()
