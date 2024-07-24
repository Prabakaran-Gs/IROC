import serial

def read_from_serial(port, baudrate):
    try:
        ser = serial.Serial(port, baudrate)
        print(f"Connected to {port} at {baudrate} baudrate")
        
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').rstrip()
                print(f"Received: {line}")
                
    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    port = "/dev/ttyACM1"  # Replace with your port
    baudrate = 115200  # Replace with your baudrate
    read_from_serial(port, baudrate)
