import time
import cv2
import numpy as np

self.polygon_coords_2 = [(150,200),  (446,200),(448,469), (148,467)]
        self.polygon_2 = Polygon(polygon_coords_2) # sample test tube align hivision

def draw_poly(self,image,polygon_coords):
          #logger.log_info("Drawing polygon")
        #polygon_coords = [(301, 1080), (591, 532), (1333, 511) , (1649, 1080)]

          # Convert the polygon coordinates to a NumPy array
        polygon_pts = np.array(polygon_coords, np.int32)
        polygon_pts = polygon_pts.reshape((-1, 1, 2))

          # Draw the polygon on the image
        cv2.polylines(image, [polygon_pts], isClosed=True, color=(0, 255, 0), thickness=3)

        return image


def test_tube(self):
        cap = cv2.VideoCapture(0)
        while True:
            command = 's'
            ret, frame = cap.read()
            
            counter+=1
            if counter%2 == 0:
                counter = 0
                continue


            if not ret:
                print("Error: Could not read frame.")
                #logger.log_error("Error: Could not read frame. test tube code")

            
            self.hivision_image = draw_poly(self.hivision_image,self.polygon_coords_2)
            cv2.imshow('ouput',self.hivision_image)
            if cv2.waitKey(1) & 0XFF == ord('q'):
                break
            flag = False
            if len(result[0]) != 0:
                point = self.get_pos(result[0].boxes)
                (x,y) = point
                point_obj = Point(point)
                text = "test tube "
                print("test tube detected")
                if self.point_within(point_obj,self.polygon_2):
                    self.ser.write('s'.encode())
                    self.ser1.write('a'.encode())
                    print("picking ")
                    text_size = cv2.getTextSize(text, self.font, self.font_scale, self.thickness)[0]
                    box = list(map(int,((result[0].boxes.xyxy).tolist()[0])))
                    x1, y1, x2, y2 = box
                    text_position = ((x1 + x2 - text_size[0]) // 2, y1 - 10)
                    #logger.log_info(f"text {text} angle {angle}")
                    cv2.putText(self.hivision_image, text, text_position, self.font, self.font_scale, self.font_color, self.thickness)
                    
                    self.cap.release()

                    start_time = time.time()
                    while True:
                        print("Working...")
                        self.ser.write('s'.encode())
                        time.sleep(0.25)
                        # Check if the timeout has been reached
                        if time.time() - start_time >= 65:
                            #logger.log_info("Timeout reached")
                            print("Timeout reached")
                            back_time = time.time()
                            while True:
                                time.sleep(0.1)
                                self.ser.write('b'.encode())
                                if time.time() - back_time >= 2:
                                    break
                        
                        if self.ispicked:
                            print("Condition met")
                            return
    
                else:
                    if x>=0 and x <= 150:
                        self.ser.write('l'.encode())
                        flag = True
                        print("left hivision test tube")
                        #logger.log_info("left hivision test tube")
                    elif x>=441 and x <=1920:
                        print("right hivision test tube")
                        flag = True
                        self.ser.write('r'.encode())

            if not flag:
                self.ser.write('f'.encode())
            else:
                self.ser.write('s'.encode())