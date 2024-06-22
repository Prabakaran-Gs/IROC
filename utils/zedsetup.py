import pyzed.sl as sl
import cv2
import time
import threading
import signal
import numpy as np

class MyZED:
    def __init__(self):
        '''
        Initialize parameters and open the ZED camera.
        '''
        self.zed = sl.Camera()
        self.left_image = sl.Mat()
        self.depth_image = sl.Mat()
        self.timestamp = 0
        self.stop_signal = False
        signal.signal(signal.SIGINT, self.signal_handler)

        self.init = sl.InitParameters()
        self.init.camera_resolution = sl.RESOLUTION.HD1080
        #print(dir(sl.RESOLUTION))
        self.init.depth_mode = sl.DEPTH_MODE.ULTRA
        self.init.camera_fps = 30  # The framerate is lowered to avoid any USB3 bandwidth issues

        # List and open the first camera
        cameras = sl.Camera.get_device_list()
        if len(cameras) == 0:
            #logger.log_info("No ZED cameras detected. exiting program")
            print("No ZED cameras detected.")
            exit(1)

        cam = cameras[0]
        self.init.set_from_serial_number(cam.serial_number)
        print("Opening ZED {}".format(cam.serial_number))
        #logger.log_info("Opening ZED {}".format(cam.serial_number))

        status = self.zed.open(self.init)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            #logger.log_error(repr(status) + "exiting program")
            self.zed.close()
            exit(1)

        self.thread = threading.Thread(target=self.grab_run)
        self.thread.start()
        time.sleep(2)

    def grab_run(self):
        '''
        Thread function to grab images from the camera.
        '''
        #logger.log_info("Thread function to grab images from the camera.")
        runtime = sl.RuntimeParameters()
        runtime.texture_confidence_threshold = 89
        runtime.remove_saturated_areas = True
        runtime.confidence_threshold = 34
        frame  = 0
        ctr = 0
        while not self.stop_signal:
            err = self.zed.grab(runtime)
            if err == sl.ERROR_CODE.SUCCESS:
                frame +=1
                self.zed.retrieve_image(self.left_image, sl.VIEW.LEFT)
                cv2.imshow("data_left",self.left_image.get_data())
                self.zed.retrieve_measure(self.depth_image, sl.MEASURE.DEPTH)
                self.timestamp = self.zed.get_timestamp(sl.TIME_REFERENCE.CURRENT).data_ns
                if frame % 60 == 0:
                    frame = 0
                    cv2.imwrite(f"data_depth/{ctr}.jpg",self.left_image.get_data())
                    cv2.imwrite(f"data_raw/{ctr}.jpg",self.depth_image.get_data())
                    ctr+=1
            else:
                pass
                #logger.log_error("Error in Fetching the Frames from Zed" + str(err))
            time.sleep(0.001) #1ms
        self.zed.close()

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
        #logger.log_info("Stopping Zed Camera signal")
        self.stop_signal = True
        time.sleep(0.5)
        exit()


    def display(self):
        '''
        Display the camera images.
        '''
        last_timestamp = 0
        key = ''
        while key != 113:  # for 'q' key
            if self.timestamp > last_timestamp:
                cv2.imshow("ZED", self.left_image.get_data())
                last_timestamp = self.timestamp
            key = cv2.waitKey(10)
        cv2.destroyAllWindows()
        self.stop_signal = True
        self.thread.join()
        print("\nFINISH")


    def get_gyro(self):
        '''
        Get the gyroscope Y-axis data from the ZED camera.
        '''
        sensor_data = sl.SensorsData()
        if self.zed.get_sensors_data(sensor_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS:
            return sensor_data.get_imu_data().get_angular_velocity()
        else:
            return None
        
    def map_range(self,value):
        in_min = -1
        in_max = 1
        out_min = 0
        out_max = 360
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        
    def get_orientation(self):
        '''
        Get the orientation data from the ZED camera's sensors.
        '''
        sensor_data = sl.SensorsData()
        #  quaternion = sensor_data.get_imu_data().get_pose().get_orientation().get()
        if self.zed.get_sensors_data(sensor_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS:
            data = sensor_data.get_imu_data().get_pose().get_orientation().get()
            orientation = list(map(self.map_range, data))

            return orientation

    def get_depth_image(self):
        depth_data = self.depth_image.get_data()
        depth_image = np.nan_to_num(depth_data, nan=0.0, posinf=0.0, neginf=0.0)
        depth_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        depth_colormap = cv2.applyColorMap(depth_image, cv2.COLORMAP_JET)
        
        return(depth_image)
    
