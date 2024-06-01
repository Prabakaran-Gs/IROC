from utils.zedsetup import MyZED
import cv2 as cv
import time

if __name__ == '__main__':

    zed = MyZED()
    
    while True:
        cv.imshow("Viw",zed.depth_image.get_data())

        if cv.waitKey(1) & 0XFF == ord('q'):
            break

    zed.stop_signal = True
