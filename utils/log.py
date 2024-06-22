import os
import datetime

class Logger:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
            cls._instance._init_instance()
        return cls._instance

    def _init_instance(self):
        self.log_folder = os.path.join("Log", datetime.datetime.now().strftime("%Y%m%d%H%M%S"))
        self.log_file = os.path.join(self.log_folder, "log.txt")
        self.image_folder = os.path.join(self.log_folder, "images")
        self.image_count = 0  # Initialize the image count

        # Create the log folder and image folder if they don't exist
        os.makedirs(self.log_folder, exist_ok=True)
        os.makedirs(self.image_folder, exist_ok=True)

    def log_info(self, info_message):
        self._log("INFO", info_message)

    def log_error(self, error_message):
        self._log("ERROR", error_message)

    def save_image(self, image_data, image_name):
        # Append the count to the image name
        image_name_with_count = f"{image_name}_{self.image_count}.jpg"
        image_path = os.path.join(self.image_folder, image_name_with_count)
        with open(image_path, "wb") as f:
            f.write(image_data)
        self.image_count += 1  # Increment the image count

    def _log(self, log_type, message):
        with open(self.log_file, "a") as f:
            timestamp = datetime.datetime.now().strftime("[%Y-%m-%d %H:%M:%S] ")
            log_entry = f"{timestamp}[{log_type}] {message}\n"
            f.write(log_entry)

# Example usage:
logger = Logger()
print("Log Instance is created ")
# logger1 = Log()
# logger1.log_info("Logger 1")
# logger2 = Log()
# logger2.log_info("Logger 2")

# print(logger1 is logger2)  # Output: True
