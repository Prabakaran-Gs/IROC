import serial
import time
n = 3
ser = serial.Serial(port='/dev/ttyUSB2',baudrate=9600,timeout=1)
if ser.isOpen():
	print(f"Serial port {ser.port} opened successfully.")
            #logger.log_info(f"Serial port {self.ser2.port} opened successfully.")
else:
	print(f"Failed to open serial port {ser.port}.")
	exit()
       
while True:
	try:
		cmd = input().strip()
		for i in range(n):
			ser.write(cmd.encode())
			time.sleep(0.1)
	except Exception as e:
		print(f"Error in Serial Thread {e}") 
		time.sleep(1)
