import serial
import time

def send_serial_data(port, baud_rate, data):
    try:
        # Open serial port
        ser = serial.Serial(port, baud_rate, timeout=1)
        time.sleep(2)  # Wait for the connection to establish

        # Send data
        ser.write(data.encode('utf-8'))
        print(f"Sent: {data}")

        # Close serial port
        ser.close()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    # Set your port and baud rate
    port = '/dev/ttyACM0'  # Change this to your port, e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux
    baud_rate = 115200       # Set the baud rate according to your device

    # Get user input
    data = input("Enter data to send: ")

    # Send data over serial
    send_serial_data(port, baud_rate, data)
