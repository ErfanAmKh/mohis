"""BMW car controller: drive forward and read Lidar/Camera/GPS/IMU."""

from vehicle import Driver
import cv2
import numpy as np

# Create the driver instance (for car-like PROTOs)
driver = Driver()

# Time step of the simulation (ms)
TIME_STEP = 16

# ------------- GET SENSORS -------------

# LIDAR
lidar = driver.getDevice("lidar")          # change name if needed
lidar.enable(TIME_STEP)

# CAMERA
camera = driver.getDevice("camera")        # change name if needed
camera.enable(TIME_STEP)

# GPS
gps = driver.getDevice("gps")              # change name if needed
gps.enable(TIME_STEP)

# INERTIAL UNIT
# If your device is named "inertial unit" in the Scene Tree, use that string instead:
# imu = driver.getDevice("inertial unit")
imu = driver.getDevice("inertial unit")
imu.enable(TIME_STEP)

# ------------- CONTROL PARAMETERS -------------

# Cruising speed is in km/h
target_speed_kmh = 5.0    
steering_angle = 0.0       

driver.setCruisingSpeed(target_speed_kmh)
driver.setSteeringAngle(steering_angle)

# ------------- MAIN LOOP -------------

while driver.step() != -1:
    # ---- Read sensors ----
    gps_values = gps.getValues()            # [x, y, z]
    rpy = imu.getRollPitchYaw()             # [roll, pitch, yaw]
    lidar_data = lidar.getRangeImage()      # list/array of distances
    image = camera.getImage()               # raw image buffer

    image = camera.getImage()
    if image:
        width = camera.getWidth()
        height = camera.getHeight()

        # Convert Webots BGRA → numpy array
        img = np.frombuffer(image, dtype=np.uint8).reshape((height, width, 4))

        # Drop alpha channel → convert BGRA → BGR
        img_bgr = img[:, :, :3]

        # Display in a window
        img_bgr = cv2.resize(img_bgr, (500, 500))
        cv2.imshow("BMW Camera", img_bgr)
        cv2.waitKey(1)

        print(lidar_data[256])

    

    # ---- Keep moving forward straight ----
    driver.setCruisingSpeed(target_speed_kmh)
    driver.setSteeringAngle(steering_angle)
