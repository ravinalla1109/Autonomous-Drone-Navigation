import rospy
from sensor_msgs.msg import LaserScan, Image
import cv2
import numpy as np

def lidar_callback(data):
    """Process LIDAR data for obstacle detection."""
    distances = np.array(data.ranges)
    if np.min(distances) < 1.0:
        rospy.loginfo("Obstacle detected!")

def camera_callback(data):
    """Process camera feed for obstacle detection."""
    frame = np.frombuffer(data.data, dtype=np.uint8).reshape((data.height, data.width, -1))
    edges = cv2.Canny(frame, 50, 150)
    cv2.imshow("Edges", edges)
    cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node("obstacle_detector")
    rospy.Subscriber("/scan", LaserScan, lidar_callback)
    rospy.Subscriber("/camera/image_raw", Image, camera_callback)
    rospy.spin()
