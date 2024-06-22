import serial
ser = serial.Serial(port='/dev/ttyUSB0',baudrate=9600,timeout=1)
import time

#import keyboard

while True:
    cmd = input().strip()
    ser.write(cmd.encode())
    time.sleep(0.1)

    if cmd == 'q':
        break

ser.close()

