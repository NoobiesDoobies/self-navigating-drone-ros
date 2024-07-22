import rclpy
import cv2
import matplotlib.pyplot as plt
import numpy as np
import os

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2

class DroneNavigator():
    def __init__(self):
        self.node = None
        
        self.depth_image = None
        self.point_cloud = None



    def compute_std_dev(self, image, kernel_size=(3, 3)):
        image_32f = np.float32(image)
        # Step 1: Compute mean        
        mean = cv2.blur(image, kernel_size)
        
        # Step 2: Compute squared mean
        squared_mean = cv2.blur(image*image, kernel_size)
        
        # Step 3: Compute variance
        var = squared_mean - mean*mean

        # Step 4: Compute standard deviation
        std_dev = np.sqrt(var)

        std_dev = np.float32(std_dev)
        # Normalize std_dev to the range [0, 255] for display
        std_dev_normalized = cv2.normalize(std_dev, None, 0, 255, cv2.NORM_MINMAX)

        # Convert to uint8 if necessary
        std_dev_image = np.uint8(std_dev_normalized)

        return std_dev, std_dev_image
    
    


    def depth_image_callback(self, msg):
        # print("Received an image!")
        # Convert ROS Image message to OpenCV image
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")

        # show image
        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)
        
        
        # Process the image using OpenCV
        # Make depth image more distinguishable
        cv_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
        cv_image = cv2.convertScaleAbs(cv_image)



        # # Make window
        # cv2.namedWindow("Depth Image", cv2.WINDOW_NORMAL)
        # # Show the image
        # cv2.imshow("Depth Image", cv_image) 
        # cv2.resizeWindow("Depth Image", 400, 300)

        # Make std image
        std_dev, std_dev_image = self.compute_std_dev(cv_image)

        cv2.namedWindow("std_dev", cv2.WINDOW_NORMAL)
        cv2.imshow("std_dev", std_dev_image)
        
        # Reconstruction and Analysis 3D Scenes page 29
        min_threshold = 0
        max_threshold = 0.03
        _,filtered = cv2.threshold(std_dev, min_threshold, max_threshold, cv2.THRESH_BINARY_INV)

        filtered = cv2.normalize(filtered, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        cv2.imshow("Filtered", filtered)




        cv2.waitKey(3)
    
    def display_point_cloud_as_heatmap(self, data, width, height):
        # Assuming data is a flat array, reshape it to 2D
        if len(data) != width * height:
            raise ValueError("Data size does not match specified width and height")
        
        data_2d = np.reshape(data, (height, width))
        
        # Normalize the data to 0-255
        normalized_data = cv2.normalize(data_2d, None, 0, 255, cv2.NORM_MINMAX)
        
        # Convert to 8-bit unsigned integer
        normalized_data = np.uint8(normalized_data)
        
        # Apply a colormap for heatmap visualization
        heatmap = cv2.applyColorMap(normalized_data, cv2.COLORMAP_JET)
        
        # Display the heatmap
        cv2.imshow("Point Cloud Heatmap", heatmap)
        cv2.waitKey(10)  # Use a small delay so the window can update properly

    def point_cloud_callback(self, msg):
        self.point_cloud = msg.data
        width = msg.width
        height = msg.height
        row_step = msg.row_step
            
        # print(np.array(self.point_cloud).shape)
        # print(width, height)
        # print(width * height)
        # display point cloud into heatmap

        # cv2.imshow("point cloud", )

    def main(self):
        rclpy.init()
        self.node = rclpy.create_node("image_processor")

        # Depth image subscriber
        self.depth_image_subscriber = self.node.create_subscription(Image, "/simple_drone/front/depth/image_raw", self.depth_image_callback, 10)
        self.pointcloud_subscriber = self.node.create_subscription(PointCloud2, "/simple_drone/front/points", self.point_cloud_callback, 10)


        rclpy.spin(self.node)

        rclpy.shutdown()

def main():
    drone_navigator = DroneNavigator()

    drone_navigator.main()

if __name__ == "__main__":
    main()