import pyzed.sl as sl
import cv2
import time
import threading
import signal
import numpy as np
from ultralytics import YOLO
from shapely.geometry import Polygon, Point, box # type: ignore
from utils.log import logger
import socket
import pickle
import struct
import serial # type: ignore

class Autonomous:

    def __init__(self):

        self.model = YOLO('best.pt')
        self.hike = YOLO('hicam.pt')

        # Define the font properties
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 1
        self.font_color = (255, 255, 255)  # white color in BGR
        self.thickness = 2

        self.polygon_coords = [(301,1080), (471,760), (1333,762) ,(1649,1080) ]
        self.polygon = Polygon(self.polygon_coords) # obstacle boundary zed

        self.polygon_coords_1 = [(809,1074), (888,0), (988,0) ,(1024,1077) ]
        self.polygon_1 = Polygon(self.polygon_coords_1) # sample test tube align zed

        self.polygon_coords_2 = [(150,280),  (446,289),(448,469), (148,467)]
        self.polygon_2 = Polygon(self.polygon_coords_2) # sample test tube align hivision

        self.ispicked = False
        self.isborder = False

        self.zed = sl.Camera()
        self.left_image = sl.Mat()
        self.depth_image = sl.Mat()
        self.hivision_image = None
        self.timestamp = 0
        self.stop_signal = False
        self.processed_image_1 = None
        # ZED 2i camera focal length (in pixels)
        self.focal_length = 1000 
        signal.signal(signal.SIGINT, self.signal_handler)

        self.init = sl.InitParameters()
        self.init.camera_resolution = sl.RESOLUTION.HD1080
        self.init.depth_mode = sl.DEPTH_MODE.ULTRA
        self.init.camera_fps = 30

        self.timeout = 60

        #check for camera
        cameras = sl.Camera.get_device_list()
        if len(cameras) == 0:
            print("No ZED cameras detected.")
            exit(1)

        cam = cameras[0]
        self.init.set_from_serial_number(cam.serial_number)
        print("Opening ZED {}".format(cam.serial_number))

        status = self.zed.open(self.init)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            self.zed.close()
            exit(1)

        time.sleep(2)
        self.ser1 = serial.Serial(port='/dev/ttyUSB1',baudrate=9600,timeout=1)
        self.ser = serial.Serial(port='/dev/ttyACM0',baudrate=9600,timeout=1)
        # Check if the serial port is open
        if self.ser.isOpen():
            print(f"Serial port {self.ser.port} opened successfully.")
        else:
            print(f"Failed to open serial port {self.ser.port}.")
            exit()
        # Check if the serial port is open
        if self.ser1.isOpen():
            print(f"Serial port {self.ser.port} opened successfully.")
        else:
            print(f"Failed to open serial port {self.ser.port}.")
            exit()
    
    def point_within(self, point,polygon):
        return polygon.contains(point)
    
    
    def hivision(self):
        print("Initializing Sample Picking")
        # Initialize the camera (0 is the default camera)
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Error: Could not open camera.")
            exit()
        
        frame_count = 0
        while not self.ispicked:
            ret, frame = cap.read()
            frame_count += 1
            if frame_count % 2 == 0:
                frame_count = 0
                continue
            if not ret:
                print("Error: Could not read frame.")

            result = self.hike.predict(frame,verbose = False,conf=0.6)
            
            self.hivision_image = result[0].plot()
            
            self.hivision_image = self.draw_poly(self.hivision_image,self.polygon_coords_2)


            
            
            for obj in result[0].boxes:
                point = self.get_pos(result[0].boxes)
                (x,y) = point
                point_obj = Point(point)
                if self.point_within(point_obj,self.polygon_2):
                    angle = self.get_angle(frame)
                    print(f'rotate angle{angle:.2f} and pick')
                    angle = int(abs(angle)%90)
                    # return pick 
                    #self.ser1.write(str(angle).encode())
                    self.ser1.write('a'.encode())
                    start_time = time.time()
                    self.ser.write('s'.encode())
                    print("Task started")
                    while True:
                        # Simulate a task (you can replace this with actual work)
                        time.sleep(1)  # Simulate work by sleeping
                        print("Working...")
			 #self.ser.write('s'.encode())
                        # Check if the timeout has been reached
                        if time.time() - start_time >= self.timeout:
                            print("Timeout reached")
                            back_time = time.time()
                            while True:
                                self.ser.write('b'.encode())
                                if time.time() - back_time >= 3:
                                    break
                            break
                        
                        # Condition to break out of the loop early
                        # Replace this condition with your actual condition
                        if self.ispicked:
                            print("Condition met")
                            cap.release()
                            # cv2.destroyAllWindows()
                            return
                else:
                    if x>=0 and x <= 150:
                        self.ser.write('l'.encode())
                                        ##print("left")
                    elif x>=441 and x <=1920:
                                        ##print("right")
                        self.ser.write('r'.encode())

            if not self.isborder:
                print('Forward')
                self.ser.write('f'.encode())
                


            
            self.hivision_image = cv2.resize(self.hivision_image,(360,240))
            cv2.imshow("Hik Vision",self.hivision_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


    def draw_poly(self,image,polygon_coords):
          #logger.log_info("Drawing polygon")
        #polygon_coords = [(301, 1080), (591, 532), (1333, 511) , (1649, 1080)]

          # Convert the polygon coordinates to a NumPy array
        polygon_pts = np.array(polygon_coords, np.int32)
        polygon_pts = polygon_pts.reshape((-1, 1, 2))

          # Draw the polygon on the image
        cv2.polylines(image, [polygon_pts], isClosed=True, color=(0, 255, 0), thickness=3)

        return image


    def within_poly(self,bbox,polygon):

        box = list(map(int,((bbox.xyxy).tolist()[0])))
        #print(box)

        x_min, y_min, x_max, y_max = box
        corners = [
               (x_min, y_min),  # top-left
               (x_max, y_min),  # top-right
               (x_max, y_max),  # bottom-right
               (x_min, y_max)   # bottom-left
          ]
          # Check if all corners are inside the polygon

        return any(polygon.contains(Point(corner)) for corner in corners)



    def get_depth(self, x, y):
        '''
        Get the depth value at the specified pixel coordinates.
        '''
        #logger.log_info("Get the depth value at the specified pixel coordinates is called")
        err, depth_value = self.depth_image.get_value(x, y)
        if np.isfinite(depth_value):



            #logger.log_info(f"Get the depth value at the specified pixel coordinates value {depth_value}")
            return depth_value
        else:
            #logger.log_info(f"Get the depth value at the specified pixel coordinates value is not finite")
            return None
    
    
    def get_pos(self,bbox):
        box = list(map(int,((bbox.xyxy).tolist()[0])))
        x_min, y_min, x_max, y_max = box

        return [(x_min+x_max)//2 , (y_min+ y_max) // 2]
    
    def get_angle(self,img):

        gray = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
        blured = cv2.GaussianBlur(gray,(5,5),0)
        canny = cv2.Canny(gray,100,200)
        denoised = cv2.fastNlMeansDenoising(canny, None, h=30, templateWindowSize=7, searchWindowSize=21)
        blurred = cv2.GaussianBlur(denoised, (7, 7), 0)
        edges = cv2.Canny(blurred, 50, 150)
        edges = cv2.dilate(edges, (7,7), iterations=5)
        edges = cv2.erode(edges, (7,7), iterations=1)

        _, binary = cv2.threshold(edges, 127, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = 7000  # Minimum area to be considered as the prominent shape
        max_area = 8000
        min_aspect_ratio = 0 

        best_contour = None
        largest_area = 0

        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > min_area and area < max_area:
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = h / w

                if min_aspect_ratio < aspect_ratio and area > largest_area:
                    largest_area = area
                    best_contour = contour
                    min_aspect_ratio = aspect_ratio

                candidates.append(contour)
        M = cv2.moments(best_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            # Handle the case where the can is empty or has an area of 0
            cx, cy = 0, 0

        if M["mu20"] != 0:
        # Calculate the angle using the central moments
            angle = 0.5 * np.arctan(2 * M["mu11"] / (M["mu20"] - M["mu02"]))
            angle = np.rad2deg(angle)
        else:
            # Handle the case where the can is a straight line or has an area of 0
            angle = 0

        return angle



    def calculate_box_area(self,bbox ,depth):
        box = list(map(int,((bbox.xyxy).tolist()[0])))
        x1, y1, x2, y2 = box

        width = abs(x2 - x1)
        width = depth * (width / 1066)

        height = abs(y2 - y1)
        height = depth * (height / 1066)
        area = width * height
        return height , width
        
        
    def start_all(self):

        runtime = sl.RuntimeParameters()

        while not self.stop_signal:
            err = self.zed.grab(runtime)
            if err == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(self.left_image, sl.VIEW.LEFT)
                self.zed.retrieve_measure(self.depth_image, sl.MEASURE.DEPTH)
                image = self.left_image.get_data()
                img = cv2.cvtColor(image,cv2.COLOR_RGBA2RGB)
                result = self.model.predict(img,verbose = False,conf = 0.50)
                clear = True

                preprocess_image = result[0].plot()#labels

                if self.isborder:
                    #print("left or right")
                    clear = False
                else:
                    for obj in result[0].boxes:
                        if obj.cls == 0:
                            if self.within_poly(obj,self.polygon):
                
                                (x,y) = self.get_pos(obj)
                                depth_value = self.get_depth(x,y)
                                text = f"cube "
                                if depth_value:
                                    H ,W = self.calculate_box_area(obj,depth_value//10)
                                    if (H >= 15 and H <= 25) or (W >= 15 and W <= 25):
                                        text += f"150x150 depth {depth_value//10}"
                                    else:
                                        text += f"300x300 depth {depth_value//10}"
                                text_size = cv2.getTextSize(text, self.font, self.font_scale, self.thickness)[0]
                                box = list(map(int,((obj.xyxy).tolist()[0])))
                                x1, y1, x2, y2 = box
                                text_position = ((x1 + x2 - text_size[0]) // 2, y1 - 10)
                               
                                cv2.putText(preprocess_image, text, text_position, self.font, self.font_scale, self.font_color, self.thickness)
                            # obstacle.append(obj.xyxy)
                                #print('left or right')
                                self.ser.write('r'.encode())
                                clear = False 
                        elif obj.cls == 1:
                            if self.within_poly(obj,self.polygon):
                
                                (x,y) = self.get_pos(obj)
                                depth_value = self.get_depth(x,y)
                                text = f"crater "
                                if depth_value:
                                    H ,W = self.calculate_box_area(obj,depth_value//10)
                                    if (H >= 15 and H <= 25) or (W >= 15 and W <= 25):
                                        text += f"200x200 depth {depth_value//10}"
                                    else:
                                        text += f"400x400 depth {depth_value//10}"
                                text_size = cv2.getTextSize(text, self.font, self.font_scale, self.thickness)[0]
                                box = list(map(int,((obj.xyxy).tolist()[0])))
                                x1, y1, x2, y2 = box
                                text_position = ((x1 + x2 - text_size[0]) // 2, y1 - 10)
                               
                                cv2.putText(preprocess_image, text, text_position, self.font, self.font_scale, self.font_color, self.thickness)
                            # obstacle.append(obj.xyxy)
                                #print('left or right')
                                self.ser.write('r'.encode())
                                clear = False 

                                
                        if not self.ispicked:
                            if obj.cls == 2:
                                (x,y) = self.get_pos(obj) #(x,y)
                                text = f"test tube "
                                if self.within_poly(obj,self.polygon_1):
                                    depth_value = self.get_depth(x,y)
                                    if depth_value:
                                        text += f"depth value {depth_value//10}"
                                        print("Depth Value ",depth_value//10)
                                        if (depth_value//10) <= 70:
                                         
                                            #print("Switch Hik Vision")
                                            print("switch to Hik Vision")
                                            self.hivision()
                                                 
                                            
                                else:
                                    clear = False
                                    if x>=0 and x <= 888:
                                        self.ser.write('l'.encode())
                                        ##print("left")
                                    elif x>=974 and x <=1920:
                                        ##print("right")
                                        self.ser.write('r'.encode())
                                text_size = cv2.getTextSize(text, self.font, self.font_scale, self.thickness)[0]
                                box = list(map(int,((obj.xyxy).tolist()[0])))
                                x1, y1, x2, y2 = box
                                text_position = ((x1 + x2 - text_size[0]) // 2, y1 - 10)
                                cv2.putText(preprocess_image, text, text_position, self.font, self.font_scale, self.font_color, self.thickness)
                        if self.ispicked:
                            if obj.cls == 3:
                                (x,y) = self.get_pos(obj) #(x,y)
                                text = f"sample tube "
                                if self.within_poly(obj,self.polygon_1):
                                    depth_value = self.get_depth(x,y) // 10
                                    if depth_value:
                                        text += f"depth value {depth_value}"
                                        print("Depth Value ",depth_value)
                                        if depth_value <= 70:
                                         
                                            #print("Switch Hik Vision")
                                            print("switch to Hik Vision")
                                            self.hivision_2()           
                                else:
                                    clear = False
                                    if x>=0 and x <= 888:
                                        self.ser.write('l'.encode())
                                        ##print("left")
                                    elif x>=974 and x <=1920:
                                        ##print("right")
                                        self.ser.write('r'.encode())
                                text_size = cv2.getTextSize(text, self.font, self.font_scale, self.thickness)[0]
                                box = list(map(int,((obj.xyxy).tolist()[0])))
                                x1, y1, x2, y2 = box
                                text_position = ((x1 + x2 - text_size[0]) // 2, y1 - 10)
                                cv2.putText(preprocess_image, text, text_position, self.font, self.font_scale, self.font_color, self.thickness)
                        
                
                if clear :
                    self.ser.write('f'.encode())
                    pass
                    #print('forward')
                            

                # self.check_is_with_in_box()
                processed_image = self.draw_poly(preprocess_image,self.polygon_coords)
                self.processed_image_1 = self.draw_poly(processed_image,self.polygon_coords_1)
                self.processed_image_1 = cv2.resize(self.processed_image_1,(1024,600))
                cv2.imshow("ouput",self.processed_image_1)
                if cv2.waitKey(1) & 0XFF == ord('q'):
                    break
            time.sleep(0.001) #1ms

        self.zed.close()
        self.stop_signal = True

    def signal_handler(self, signal, frame):
        self.stop_signal = True
        time.sleep(0.5)
        exit()



if __name__ == '__main__':
    
    nicokla = Autonomous()
    nicokla.start_all()
    #nicokla.hivision()
    
