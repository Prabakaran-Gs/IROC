import threading
import time
import cv2
from utils.zedsetup import MyZED
from ultralytics import YOLO
from shapely.geometry import Polygon, Point, box # type: ignore
import pyzed.sl as sl
import signal
import serial # type: ignore
import numpy as np

class Autonomous:
    def __init__(self):
        #logger.log_info("Initialization of Autonomous")
        self.model = YOLO('best.pt') #zed
        self.hike = YOLO('best_100.pt') #hik vision

        # Define the font properties
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 1
        self.font_color = (255, 255, 255)  # white color in BGR
        self.thickness = 2

        self.polygon_coords = [(130,900), (380,650), (1655,650) ,(1930,900) ]
        self.polygon = Polygon(self.polygon_coords) # obstacle boundary zed

        self.polygon_coords_1 = [(905,1080), (1030,0) ,(1130,0), (1143,1080)]
        # [(840,1080), (820,0), (1060,0) ,(1040,1080) ]
        self.polygon_1 = Polygon(self.polygon_coords_1) # sample test tube align zed

        self.polygon_coords_2 = [(300,160), (376,160),(376,300), (300,300)]
        self.polygon_2 = Polygon(self.polygon_coords_2) # sample test tube align hivision

        self.commanded = 'f'
        self.ispicked = False
        self.isborder = False
        self.state = 0
        self.message = ""

        self.zed = sl.Camera()
        self.left_image = sl.Mat()
        self.depth_image = sl.Mat()
        self.hivision_image = None
        self.processed_image = None
        self.timestamp = 0
        self.stop_signal = False

        # ZED 2i camera focal length (in pixels)
        self.focal_length = 1000
        signal.signal(signal.SIGINT, self.signal_handler)

        self.init = sl.InitParameters()
        self.init.camera_resolution = sl.RESOLUTION.HD1080
        self.init.depth_mode = sl.DEPTH_MODE.ULTRA
        self.init.camera_fps = 25



        #check for camera
        cameras = sl.Camera.get_device_list()
        if len(cameras) == 0:
            print("No ZED cameras detected.")
            #logger.log_info("No ZED cameras detected.")
            exit(1)

        cam = cameras[0]
        self.init.set_from_serial_number(cam.serial_number)
        print("Opening ZED {}".format(cam.serial_number))
        #logger.log_info("Opening ZED {}".format(cam.serial_number))

        status = self.zed.open(self.init)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            #logger.log_error(repr(status))
            self.zed.close()
            exit(1)
        
        self.cap = cv2.VideoCapture(0)
        time.sleep(2)

        
        self.ser = serial.Serial(port='/dev/ttyACM0',baudrate=9600,timeout=1)
        self.ser1 = serial.Serial(port='/dev/ttyUSB0',baudrate=9600,timeout=1)
        self.ser2 = serial.Serial(port='/dev/ttyUSB2',baudrate=9600,timeout=1)
        #Check if the serial port is open
        if self.ser.isOpen():
            print(f"Serial port {self.ser.port} opened successfully.")
            #logger.log_info(f"Serial port {self.ser.port} opened successfully.")
        else:
            print(f"Failed to open serial port {self.ser.port}.")
            #logger.log_error(f"Failed to open serial port {self.ser.port}.")
            exit()
        # Check if the serial port is open
        if self.ser1.isOpen():
            print(f"Serial port {self.ser1.port} opened successfully.")
            #logger.log_info(f"Serial port {self.ser1.port} opened successfully.")
        else:
            print(f"Failed to open serial port {self.ser1.port}.")
            #logger.log_error(f"Failed to open serial port {self.ser1.port}.")
            exit()
        # Check if the serial port is open
        if self.ser2.isOpen():
            print(f"Serial port {self.ser2.port} opened successfully.")
            #logger.log_info(f"Serial port {self.ser2.port} opened successfully.")
        else:
            print(f"Failed to open serial port {self.ser2.port}.")
            #logger.log_error(f"Failed to open serial port {self.ser2.port}.")
            exit()
        
        self.thread = threading.Thread(target=self.serial_read)
        self.thread.start()


    def serial_read(self):

        while True:
            try:
                data = self.ser2.readline().decode().strip().split(',')
                #print(f"Thread data {data}")
                if len(data) == 2:
                    self.isborder,self.ispicked = list(map(int,data))
                    #print(data)
                time.sleep(0.1)
            
            except Exception as e:
                print(f"Error in Serial Thread {e}")
                #logger.log_error(f"Error in Serial Thread {e}")



    def get_decision(self):
        if self.state == 1:
            self.commanded = 'r'
            self.message = "from get_decision"
        else:
            self.commanded = 'l'
            self.message = "from get_decision"
        
        
    
    def get_pos(self,bbox):
        box = list(map(int,((bbox.xyxy).tolist()[0])))
        x_min, y_min, x_max, y_max = box

        return [(x_min+x_max)//2 , (y_min+ y_max) // 2]

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

    def calculate_box_area(self,bbox ,depth):
        box = list(map(int,((bbox.xyxy).tolist()[0])))
        x1, y1, x2, y2 = box

        width = abs(x2 - x1)
        width = depth * (width / 1066)

        height = abs(y2 - y1)
        height = depth * (height / 1066)
        area = width * height
        return height , width

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
    
    def signal_handler(self, signal, frame):
        self.stop_signal = True
        time.sleep(0.5)
        exit()

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
        command = 's'
        print("Initializing Sample Picking")
        #logger.log_info("Initializing Sample Picking")

        if not self.cap.isOpened():
            print("Error: Could not open camera.")
            #logger.log_error("Error: Could not open camera. test tube code")
            exit()
        
        self.ser.write(command.encode())
        time.sleep(1)
        counter = 1
        while not self.ispicked:
            command = 's'
            ret, frame = self.cap.read()
            
            counter+=1

            if counter%2 == 0:
                counter = 0
                continue


            if not ret:
                print("Error: Could not read frame.")
                #logger.log_error("Error: Could not read frame. test tube code")
            
            self.ser.write(command.encode())
            result = self.hike.predict(frame,verbose = False,conf=0.5)
            self.hivision_image = result[0].plot()
            
            self.hivision_image = self.draw_poly(self.hivision_image,self.polygon_coords_2)
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
                                if self.ispicked:
                                    print("Condition met")
                                    return
                                time.sleep(0.1)
                                self.ser.write('b'.encode())
                                if time.time() - back_time >= 2:
                                    return
                        
                        
    
                else:
                    if x>=0 and x <= 300:
                        self.ser.write('l'.encode())
                        flag = True
                        print("left hivision test tube")
                        #logger.log_info("left hivision test tube")
                    elif x>=376 and x <=1920:
                        print("right hivision test tube")
                        flag = True
                        self.ser.write('r'.encode())

            if not flag:
                self.ser.write('f'.encode())
            else:
                self.ser.write('s'.encode())
    
    
    def point_within(self, point,polygon):
        return polygon.contains(point)
    
    def sample_container(self):

        print("sample_container detected")
        s = time.time()
        while time.time() - s <= 1:
            self.ser.write('f'.encode())
            time.sleep(0.1)
        
        time.sleep(0.5)
        self.ser1.write('o'.encode())
        time.sleep(0.5)
        print("backward")
        s = time.time()
        while time.time() - s <= 5:
            self.ser.write('b'.encode())
            time.sleep(0.1)
        
        self.ser.write('s'.encode())
        print("Autonomous mission successfully")
        print("bye")
        self.thread.join()
        exit(0)

        
    
    def start_all(self):

        runtime = sl.RuntimeParameters()
        process_time = 0
        while not self.stop_signal:
            start_time = time.time()
            err = self.zed.grab(runtime)
            if err == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(self.left_image, sl.VIEW.LEFT)
                self.zed.retrieve_measure(self.depth_image, sl.MEASURE.DEPTH)
                image = self.left_image.get_data()
                img = cv2.cvtColor(image,cv2.COLOR_RGBA2RGB)
                result = self.model.predict(img,verbose = False,conf = 0.50)
                self.processed_image = result[0].plot() #labels
                self.processed_image = self.draw_poly(self.processed_image,self.polygon_coords)
                self.processed_image = self.draw_poly(self.processed_image,self.polygon_coords_1)
                
            self.commanded = 'f'
            self.message = "default forward"
            if self.isborder:
                self.state = 1 - self.state
                print(f"border is detected {self.isborder}")
                #self.get_decision()
                self.commanded = 'r'
                print(f"border is detected {self.isborder} and direction {self.commanded}")
                s = time.time()
                while( time.time() - s <= 5):
                    self.ser.write(self.commanded.encode())
                    time.sleep(0.1)
                self.commanded = 'f'
                
            
            elif len(result[0]) != 0:
                print("object is detected ")
                
                decision_list = []
                for obj in result[0].boxes:
                    if obj.cls == 0 or obj.cls == 1: #cube and crater
                        if self.within_poly(obj,self.polygon):
                            
                            (x,y) = self.get_pos(obj)
                            depth_value = self.get_depth(x,y)
                            print(f"object detected within poly {depth_value}")
                            if obj.cls == 0:

                                text = "cube"
                                if depth_value:
                                    H ,W = self.calculate_box_area(obj,depth_value//10)
                                    if (H >= 15 and H <= 25) or (W >= 15 and W <= 25):
                                        text += f"150x150 depth {depth_value//10}"
                                        print(text)
                                    else:
                                        text += f"300x300 depth {depth_value//10}"
                                        decision_list.append([obj,depth_value//10])
                                        print(text)
                                else:
                                    decision_list.append([obj,999])
                            else:
                                text = "crater"
                                if depth_value:
                                    H ,W = self.calculate_box_area(obj,depth_value//10)
                                    if (H >= 15 and H <= 25) or (W >= 15 and W <= 25):
                                        text += f"200x200 depth {depth_value//10}"
                                        print(text)
                                    else:
                                        text += f"400x400 depth {depth_value//10}"
                                        decision_list.append([obj,depth_value//10])
                                        print(text)
                                else:
                                    decision_list.append([obj,999])
                        
                    else:
                        if not self.ispicked and obj.cls == 2:
                            (x,y) = self.get_pos(obj)
                            depth_value = self.get_depth(x,y)
                            if depth_value:
                                decision_list.append([obj,depth_value//10])
                            else:
                                decision_list.append([obj,999])
                            
                        else:
                            (x,y) = self.get_pos(obj)
                            depth_value = self.get_depth(x,y)
                            if depth_value:
                                decision_list.append([obj,depth_value//10])
                            else:
                                decision_list.append([obj,999])
                
                decision_list.sort(key = lambda x : x[1])

                #print(decision_list)
                if len(decision_list):
                    current = decision_list[0][0]
                    if current.cls == 0 or current.cls == 1:
                        #self.get_decision()
                        self.commanded = 'r'
                    elif not self.ispicked and current.cls == 2:
                        point = self.get_pos(current) #(x,y)
                        (x,y) = point
                        point_obj = Point(point)
                        depth_value = self.get_depth(x,y)
                        text = f"test tube "
                        
                        if self.point_within(point_obj,self.polygon_1):
                            print("test tube within range")
                            if depth_value:
                                text += f"depth value {depth_value//10}"
                                print("Depth Value ",depth_value//10)
                                if (depth_value//10) <= 70:
                                    #print("Switch Hik Vision")
                                    print("switch to Hik Vision for test tube")
                                    self.ser.write('s'.encode())
                                    self.test_tube()
                                else:
                                    print("Aligned Forward")
                                    self.commanded = 'f'
                                    self.message = "aligned test tube forward"
                            else:
                                print("Depth not found Test Tube Forward")
                                self.message = "Depth not found Test Tube Forward"
                                self.commanded = 'f'
                        else:
                            print("test tube align")
                            if x>=0 and x <= 1030:
                                self.commanded = 'l'
                                print("left test tube ")
                                self.message = "left test tube "
                            elif x>=1130 and x <=1920:
                                print("right test tube ")
                                self.commanded = 'r'
                                self.message = "right test tube "
                    
                    elif self.ispicked and current.cls == 3:
                        point = self.get_pos(current) #(x,y)
                        point_obj = Point(point)
                        (x,y) = point

                        depth_value = self.get_depth(x,y)
                        text = f"sample container "
                        if self.point_within(point_obj,self.polygon_1):
                            print("sample container within range")
                            if depth_value:
                                text += f"depth value {depth_value//10}"
                                print("Depth Value ",depth_value//10)
                                if (depth_value//10) <= 70:
                                    #print("Switch Hik Vision")
                                    print("switch to Hik Vision for sample container")
                                    self.commanded = 's'
                                    self.ser.write('s'.encode())
                                    self.sample_container()
                                else:
                                    print("Aligned Forward")
                                    self.commanded = 'f'
                                    self.message = "algined container forward "
                        else:
                            print("sample container align")
                            if x>=0 and x <= 1030:
                                self.commanded = 'l'
                                print("left sample container")
                                self.message = "sample container align left "
                            elif x>=1130 and x <=1920:
                                print("right sample container ")
                                self.commanded = 'r'
                                self.message = "sample container align right"                   
                else:
                    self.commanded = 'f'
                    self.message = "no sample container forward"
            
            else:
                self.commanded = 'f'
                self.message = "no detection forward"

            #print(f"Send Command {self.commanded}")
            #print(self.message)
            self.ser.write(self.commanded.encode())
            process_time = time.time() - start_time

            print(f"Time taken to process {process_time}s")
            self.processed_image = cv2.resize(self.processed_image,(720,360))
            cv2.imshow("ouput",self.processed_image)
            if cv2.waitKey(1) & 0XFF == ord('q'):
                break
            time.sleep(0.001)



if __name__ == '__main__':
    
    nicokla = Autonomous()
    nicokla.start_all()
    #nicokla.test_tube()

