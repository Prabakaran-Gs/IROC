import serial
import time
ser2 = serial.Serial(port='/dev/ttyUSB2',baudrate=9600,timeout=1)
if ser2.isOpen():
	print(f"Serial port {ser2.port} opened successfully.")
            #logger.log_info(f"Serial port {self.ser2.port} opened successfully.")
else:
	print(f"Failed to open serial port {self.ser2.port}.")
	exit()
       
while True:
	try:
		data = ser2.readline().decode().strip().split(',')
		print(f"Thread data {data}")
		if len(data) == 2:
			self.isborder,self.ispicked = list(map(int,data))
		time.sleep(0.1)
	except Exception as e:
		print(f"Error in Serial Thread {e}") 
		time.sleep(1)
