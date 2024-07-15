import rclpy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def image_callback(msg):
    print("Received an image!")
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

    # Show the image
    cv2.imshow("Depth Image", cv_image)
    cv2.waitKey(3)

def main():
    rclpy.init()
    node = rclpy.create_node("image_subscriber")

    # Create a subscriber
    subscriber = node.create_subscription(Image, "/simple_drone/front/depth/image_raw", image_callback, 10)

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()