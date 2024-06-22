import pyzed.sl as sl

def main():
    # Create a Camera object
    zed = sl.Camera()

    # Set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 video mode
    init_params.coordinate_units = sl.UNIT.METER  # Set units in meters

    # Open the camera
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open the ZED camera")
        zed.close()
        return

    # Get camera information (serial number, firmware version, etc.)
    camera_info = zed.get_camera_information()
    print("Camera serial number: ", camera_info.serial_number)

    # Create a SensorsData object
    sensors_data = sl.SensorsData()

    try:
        while True:
            # Retrieve the current sensor data
            if zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS:
                # Get the IMU data (accelerometer, gyroscope, etc.)
                imu_data = sensors_data.get_imu_data()

                # Extract accelerometer values
                acceleration = imu_data.get_linear_acceleration()
                print(f"Accelerometer values: x={acceleration[0]:.2f}, y={acceleration[1]:.2f}, z={acceleration[2]:.2f}")

    except KeyboardInterrupt:
        # Stop the loop when the user presses Ctrl+C
        print("Stopping...")

    # Close the camera
    zed.close()

if __name__ == "__main__":
    main()
