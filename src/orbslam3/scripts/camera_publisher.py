#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')

        # Replace with your phone's MJPEG stream URL
        video_url = 'http://10.19.42.168:4747/video'
        self.cap = cv2.VideoCapture(video_url)

        if not self.cap.isOpened():
            self.get_logger().error("Error: Could not open video stream")
            return

        self.publisher_ = self.create_publisher(Image, 'camera', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn("Warning: Couldn't retrieve frame. Check the stream URL.")
            return
        
        cv2.imshow("Camera", frame)
        cv2.waitKey(1)

        # msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        # msg.header.stamp = self.get_clock().now().to_msg()  # Add timestamp
        # self.publisher_.publish(msg)
        

    def destroy_node(self):
        self.cap.release()  # Release the video capture when shutting down
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
