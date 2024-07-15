import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def image_callback(msg):
    # Convert ROS Image message to OpenCV image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # Process the image using OpenCV
    # ...

def main():
    rospy.init_node("path_finder")
    rospy.Subscriber("/camera/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == "__main__":
    main()