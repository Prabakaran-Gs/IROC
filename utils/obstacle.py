from zedsetup import MyZED
import threading
import time
import serial

class Border:
     
     def __init__(self) :
          self.is_border = False
          self.port = '/dev/ttyUSB0'
          self.ser = serial.Serial(
          port= self.port,       # Replace with your port name
          baudrate=9600,             # Set baudrate to your device's baudrate
          timeout=1                  # Read timeout in seconds
          )
          for i in range(3):
               try:
                    if not self.ser.is_open:
                         self.ser.open()
                    self.thread = threading.Thread(target=self.check_border)
                    self.thread.start()
                    break 
               except:
                    print(f"Serial Monitor can be opened {self.port}") 

               time.sleep(2)        

     def check_border(self):
          '''
          returns True if border detected
          '''
          try:
               while True:
                    # Read a line of input from the serial port
                    line = self.ser.readline().decode('utf-8').rstrip()
                    if line == "border":
                         self.is_border = True
                    else:
                         self.is_border = False        
          except Exception as e:
               print(e)



def is_box_crater():
     '''
     returns True 
     '''
     pass

def already_visited():
     '''
     returns True
     '''
     pass