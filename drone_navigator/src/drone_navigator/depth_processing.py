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

        self.intrinsic_matrix = {
            "fx": 572.5741169395627,
            "cx": 320.5,
            "fy": 572.5741169395627,
            "cy": 240.5
        }

        self.std_cutoff = 0.03
        cv2.namedWindow("Filtered by Range uint8")
        # Make slider for std cutoff value
        cv2.createTrackbar("Std Cutoff", "Filtered by Range uint8", 0, 100, self.update_std_cutoff)




    def compute_std_dev(self, image, kernel_size=(3, 3)):
        # Step 1: Compute mean        
        mean = cv2.blur(image, kernel_size)
        
        # Step 2: Compute squared mean
        squared_mean = cv2.blur(image*image, kernel_size)
        
        # Step 3: Compute variance
        var = squared_mean - mean*mean

        # Step 4: Compute standard deviation
        # std_dev = np.sqrt(var)
        std_dev = var

        std_dev = np.float32(std_dev)
        # Normalize std_dev to the range [0, 255] for display
        std_dev_normalized = cv2.normalize(std_dev, None, 0, 255, cv2.NORM_MINMAX)

        # Convert to uint8 if necessary
        std_dev_image = np.uint8(std_dev_normalized)

        return std_dev, std_dev_image
    
    
    def depth_to_xyz(self, depth_image, fx, fy, cx, cy):
        height, width = depth_image.shape
        x = np.arange(0, width)
        y = np.arange(0, height)
        xx, yy = np.meshgrid(x, y)
        
        # Convert from image coordinates to camera coordinates
        X = (xx - cx) * depth_image / fx
        Y = (yy - cy) * depth_image / fy
        Z = depth_image
        
        # Stack to get 3D coordinates
        xyz = np.stack((X, Y, Z), axis=-1)
        
        return xyz

    def filter_by_range_relibility(self, depth_image, kernel_size=(3,3), max_std = 0.03):
        depth_image_copy = depth_image.copy()

        # Make std image
        std_dev, std_dev_image = self.compute_std_dev(depth_image, kernel_size)

        # Reconstruction and Analysis 3D Scenes page 29, suggests max std at 0.03
        max_threshold = max_std
        _,filter = cv2.threshold(std_dev, max_threshold, 1, cv2.THRESH_BINARY_INV)

        # Filtered by range image
        filtered_by_range = np.multiply(depth_image_copy, filter)
        filtered_by_range_image = cv2.normalize(filtered_by_range, None, 0, 255, cv2.NORM_MINMAX)
        
        
        return filtered_by_range, filtered_by_range_image
    
    def update_std_cutoff(self, value):
        self.std_cutoff = value/100

    def depth_image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
              
        # Make depth image more distinguishable
        cv_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)


        # Filter by range reliability
        filtered_by_range, filtered_by_range_image = self.filter_by_range_relibility(cv_image, kernel_size=(3, 3), max_std=self.std_cutoff)
        # cv2.imshow("Filter", filtered_by_range)
        # cv2.imshow("Filtered by Range", filtered_by_range_image)


        # Display the original image last because I don't want to convert it to uint8 before calculation
        cv_image = cv2.convertScaleAbs(cv_image)
        # create window for displaying image using
        cv2.imshow("Image window", cv_image)

        filtered_by_range_uint8 = cv2.convertScaleAbs(filtered_by_range)
        cv2.imshow("Filtered by Range uint8", filtered_by_range_uint8)

        cv2.waitKey(3)

    def point_cloud_callback(self, msg):
        self.point_cloud = msg.data
        width = msg.width
        height = msg.height
        row_step = msg.row_step

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