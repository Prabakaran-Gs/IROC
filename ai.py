import threading
import time
import cv2
from ultralytics import YOLO
from shapely.geometry import Polygon, Point
import pyzed.sl as sl
import signal
import serial
import numpy as np

# costom imports
from helper import *

# Constants
ZED_MODEL_FILE = "zed_model.pt"
HIKE_VISION_MODEL_FILE = "hike_model.pt"

CONTROLLER_PORT = "/dev/ttyACM0"
SENSOR_DATA_PORT = "/dev/ttyACM1"

HIKE_VISION_CAMERA_INDEX = 2

# Terminator
stop_signal = False

class Autonomous:

    def __init__(self):

        self.zed_model = YOLO(ZED_MODEL_FILE)
        self.hikevison_model = YOLO(HIKE_VISION_MODEL_FILE)

        self.font_property()
        self.declare_poly_coords()

        # Parameters 
        self.command = 'f'
        self.is_picked = False
        self.is_border = False
        self.log_msg = ""

        # Camera Parameters
        self.zed = sl.Camera()
        self.left_image = sl.Mat()
        self.depth_image = sl.Mat()
        self.processed_image = None
        self.hivision_image = None
       

        # Camera Settings 
        self.setup_zed()
        self.setup_hike()

        # Thread
        self.setup_slaves()

    def font_property(self):    
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 1
        self.font_color = (255, 255, 255)  # white color in BGR
        self.thickness = 2

    def declare_poly_coords(self):

        self.obstacle_region_coords = [(130,900), (380,650), (1655,650) ,(1930,900) ]
        self.align_region_coords = [(905,1080), (1030,0) ,(1130,0), (1143,1080)]
        self.picking_region_coords = [(275,307), (275,170),(320,170), (320,307)]

        self.obstacle_region = Polygon(self.obstacle_region_coords)
        self.align_region = Polygon(self.align_region_coords)
        self.picking_region = Polygon(self.picking_region_coords)

    def signal_handler(self, signal, frame):
        global stop_signal
        stop_signal = True
        time.sleep(0.5)
        exit()

    def setup_zed(self):
        
        signal.signal(signal.SIGINT, self.signal_handler)

        # ZED Parameter
        self.focal_length = 1000
        self.init = sl.InitParameters()
        self.init.camera_resolution = sl.RESOLUTION.HD1080
        self.init.depth_mode = sl.DEPTH_MODE.ULTRA
        self.init.camera_fps = 25
        
        # Camera Check
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

    def setup_hike(self):

        self.hikevision_camera = cv2.VideoCapture(HIKE_VISION_CAMERA_INDEX)
        if self.hikevision_camera is None or not self.hikevision_camera.isOpened():
           print("Hike Vision Camera is not found...!")
        else:
            print("Hike Vision Connected...!")
        self.hikevision_camera.release()

    def setup_slaves(self):

        self.controller = serial.Serial(port='/dev/ttyACM0',baudrate=115200,timeout=1)
        self.sensor_data = serial.Serial(port='/dev/ttyACM1',baudrate=115200,timeout=1)

        if self.controller.isOpen():
            print("Controller Connected ...")
        else:
            print(f"Failed to open serial port {self.ser.port}.")
            exit()

        if self.sensor_data.isOpen():
            print("Sensor Data Connected ...")
        else:
            print(f"Failed to open serial port {self.ser.port}.")
            exit()
        
        self.sensor_thread = threading.Thread(target=self._start_sensor_data)
        self.sensor_thread.start()

    def _start_sensor_data(self):
        global stop_signal

        while not stop_signal:
            try:
                data = self.sensor_data.readline().decode().strip().split(',')
                if len(data) == 2:
                    self.is_border, _ = list(map(int,data))
                time.sleep(0.1)
            
            except Exception as e:
                print(f"Error in Serial Thread {e}")


def nikola():
    # Terminator
    global stop_signal

    runtime = sl.RuntimeParameters()
    rover = Autonomous()
    
    while not stop_signal :
        start_time = time.time()
        err = rover.zed.grab(runtime)

        # Read Image and Draw Polygons
        if err == sl.ERROR_CODE.SUCCESS:
            rover.zed.retrieve_image(rover.left_image, sl.VIEW.LEFT)
            rover.zed.retrieve_measure(rover.depth_image, sl.MEASURE.DEPTH)
            image = rover.left_image.get_data()
            img = cv2.cvtColor(image,cv2.COLOR_RGBA2RGB)

            result = rover.zed_model(img,verbose = False,conf = 0.50)
            rover.processed_image = result[0].plot() #labels
            rover.processed_image = draw_poly(rover.processed_image, rover.obstacle_region_coords)
            rover.processed_image = draw_poly(rover.processed_image,rover.align_region_coords)

        resized_img = cv2.resize(rover.processed_image,(960,540))
        cv2.imshow("telementry",resized_img)
        if cv2.waitKey(1) & 0XFF == ord('q'):
            stop_signal = True
            rover.sensor_thread.join()
            cv2.destroyAllWindows()
            exit(0)   

        # Main Operations
        rover.command = "f"
        rover.log_msg = "Default Movement"


        if rover.is_border : #Border Checks
            '''
                Turn Right and Move Forward
            '''
            rover.command = "Border Detected"
            rover.command = "r"

            # Turn right for 5 seconds 
            print("Border Detected")
            start_turn = time.time()
            while (time.time() - start_turn <= 7):
                rover.controller.write(rover.command.encode())
                time.sleep(0.1)
                rover.controller.write('s'.encode())
                time.sleep(0.1)
            # Move forward
            # start_time = time.time()
            # while time.time() - start_time < 2:
            #     rover.controller.write('f'.encode())
            rover.command = 'f'

        else:
            closest_object = get_closest_obj(result,rover.depth_image,rover.obstacle_region,rover.is_picked)
            if closest_object:
                if closest_object[0].cls == 0 or closest_object[0] == 1:
                    rover.command = "r"
                    rover.log_msg = "Obstacle detected"

                if closest_object[0].cls == 2: #Test Tube
                    # Parameters
                    point = get_pos(closest_object[0])
                    (x,y) = point
                    point = Point(point)
                    depth = closest_object[1]

                    if point_within(point,rover.align_region):
                        if depth <= 70:
                            print("Switching to Hike Vision")
                            test_tube(rover)

                        else:
                            rover.log_msg = "Test Tube Aligned"
                            rover.command = "f"

                    else:
                        rover.log_msg = "Aligning Test Tube"
                        rover.command = get_align_cmd(x,y)

                if closest_object[0].cls == 3:
                    # Parameters
                    point = get_pos(closest_object[0])
                    (x,y) = point
                    point = Point(point)
                    depth = closest_object[1]

                    if point_within(point,rover.align_region):
                        if depth <= 80:
                            print("Switching to Hike Vision")
                            sample_container(rover)

                        else:
                            rover.log_msg = "Sample Container Aligned"
                            rover.command = "f"

                    else:
                        rover.log_msg = "Aligning Sample Container"
                        rover.command = get_align_cmd(x,y)      

        end_time = time.time()
        process_time =  end_time - start_time
        rover.controller.write(rover.command.encode())
        print(f"[{process_time}] {rover.log_msg} -> {rover.command}")       


def test_tube(rover : Autonomous):
    command = 's'
    print("Initializing Sample Picking")
    rover.hikevision_camera = cv2.VideoCapture(HIKE_VISION_CAMERA_INDEX)
    if not rover.hikevision_camera.isOpened():
        print("Error: Could not open camera.")
        exit()
    
    rover.controller.write(command.encode())
    time.sleep(1)
    while not rover.is_picked and rover.hikevision_camera.isOpened():
        command = 's'    
        ret, frame = rover.hikevision_camera.read()
        if not ret:
            print("Error: Could not read frame.")
        
        rover.controller.write(command.encode())
        result = rover.hikevison_model.predict(frame,verbose = False,conf=0.5,classes = 0)
        rover.hivision_image = result[0].plot()
        
        rover.hivision_image = draw_poly(rover.hivision_image, rover.picking_region_coords)
        cv2.imshow('Hike Vison',rover.hivision_image)
        if cv2.waitKey(1) & 0XFF == ord('q'):
            break

        flag = False
        if len(result[0]) != 0:
            point = get_pos(result[0].boxes)
            (x,y) = point
            point_obj = Point(point)
            if point_within(point_obj,rover.picking_region):
                rover.controller.write('s'.encode())
                time.sleep(0.1)
                rover.controller.write('a'.encode())
                print("picking ")

                ''' new version'''
                start_time = time.time()
                while not rover.is_picked:
                    print("Working...")
                    rover.controller.write('s'.encode())
                    time.sleep(0.25)
                    while time.time() - start_time < 65:
                        print("Picking.......")
                        if rover.is_picked :
                            rover.hikevision_camera.release()
                            cv2.destroyWindow("Hike Vison")
                            break

                    '''edited'''
                    
                    rover.hikevision_camera.release()
                    cv2.destroyWindow("Hike Vison")
                    rover.is_picked = True
                    break

                    '''edited end'''

                    if not rover.is_picked :
                        start_time = time.time()

                        while time.time() - start_time < 1:
                            rover.controller.write('b'.encode())
                            rover.hikevision_camera.release()
                            cv2.destroyWindow("Hike Vison")                        
                        break

                    '''new version end'''

            else:
                if x>=0 and x <= 275:
                    rover.controller.write('L'.encode())
                    flag = True
                    print("left hivision test tube")
                    #logger.log_info("left hivision test tube")
                elif x>=320 and x <=1920:
                    print("right hivision test tube")
                    flag = True
                    rover.controller.write('R'.encode())
                else:
                    print("forward hivison aligned")
                    rover.controller.write('F'.encode()) 
                    ''' UPDATED '''
        else:
            print("Forward -> not Found")
            rover.controller.write('F'.encode())


def sample_container(rover : Autonomous):
    command = 's'
    print("Initializing Sample Droping")
    rover.hikevision_camera = cv2.VideoCapture(HIKE_VISION_CAMERA_INDEX)
    if not rover.hikevision_camera.isOpened():
        print("Error: Could not open camera.")
        exit()

    rover.controller.write(command.encode())
    time.sleep(1)

    while True :
        command = 's'
        
        ret, frame = rover.hikevision_camera.read()
        if not ret:
            print("Error: Could not read frame.")
            #logger.log_error("Error: Could not read frame. test tube code")
        
        rover.controller.write(command.encode())
        result = rover.hikevison_model.predict(frame,verbose = False,conf=0.5,classes = 1)
        rover.hivision_image = result[0].plot()
        
        rover.hivision_image = draw_poly(rover.hivision_image, rover.picking_region_coords)
        cv2.imshow('Hike Vison',rover.hivision_image)
        if cv2.waitKey(1) & 0XFF == ord('q'):
            break

        flag = False
        if len(result[0]) != 0:
            point = get_pos(result[0].boxes)
            (x,y) = point
            point_obj = Point(point)

            if point_within(point_obj,rover.picking_region):
                rover.controller.write('o'.encode())
                time.sleep(0.1)
                print("Droped Successfully ")

                s = time.time()
                while time.time() - s <= 5:
                    rover.controller.write('b'.encode())
                    time.sleep(0.1)

                rover.controller.write('s'.encode())
                print("Autonomous mission successfully")
                print("bye")
                global stop_signal
                stop_signal = True
                rover.sensor_thread.join()
                exit(0)

            else:
                if x>=0 and x <= 275:
                    rover.controller.write('L'.encode())
                    flag = True
                    print("left hivision test tube")
                    #logger.log_info("left hivision test tube")
                elif x>=320 and x <=1920:
                    print("right hivision test tube")
                    flag = True
                    rover.controller.write('R'.encode())
                else:
                    print("forward hivison aligned")
                    rover.controller.write('F'.encode())
        else:
            print("Forward -> not Found")
            rover.controller.write('F'.encode())
            


if __name__ == "__main__":
    nikola()
