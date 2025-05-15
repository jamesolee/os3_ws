#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
from djitellopy import Tello
import threading
import cv2
import math

class TelloPublisher(Node):
    def __init__(self):
        super().__init__('tello_publisher')
        # cv2.namedWindow("Controls")

        self.pub_imu = self.create_publisher(Imu, '/imu', 1)
        self.pub_camera = self.create_publisher(Image, '/camera', 10)
        self.bridge = CvBridge()

        self.drone = Tello()
        self.drone.connect()
        self.drone.streamon()

        # Set up a periodic timer to publish frames
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        # Start a separate thread for keyboard controls
        self.control_thread = threading.Thread(target=self.keyboard_control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

        self.clock = Clock()

    def get_orientation_quaternion(self):
        """
        https://github.com/tentone/tello-ros2/blob/main/workspace/src/tello/tello/node.py
        """
        deg_to_rad = math.pi / 180.0
        return euler_to_quaternion([
            self.drone.get_yaw() * deg_to_rad,
            self.drone.get_pitch() * deg_to_rad,
            self.drone.get_roll() * deg_to_rad
        ])

    def timer_callback(self):
        frame = self.drone.get_frame_read().frame
        
        if frame is not None:
            frame = cv2.resize(frame,(640,480))
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.clock.now().to_msg()
            self.pub_camera.publish(msg)

            q = self.get_orientation_quaternion()
            msg = Imu()
            msg.header.stamp = self.clock.now().to_msg()
            msg.linear_acceleration.x = self.drone.get_acceleration_x() / 100.0
            msg.linear_acceleration.y = self.drone.get_acceleration_y() / 100.0
            msg.linear_acceleration.z = self.drone.get_acceleration_z() / 100.0
            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]
            self.pub_imu.publish(msg)

    def keyboard_control_loop(self):
        while rclpy.ok():
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                self.get_logger().info('Landing and exiting...')
                self.drone.land()
                rclpy.shutdown()
                break
            elif key == ord('t'):
                self.drone.takeoff()
            elif key == ord('l'):
                self.drone.land()
            elif key == ord('w'):
                self.drone.move_forward(30)
            elif key == ord('s'):
                self.drone.move_back(30)
            elif key == ord('a'):
                self.drone.move_left(30)
            elif key == ord('d'):
                self.drone.move_right(30)
            elif key == ord('u'):
                self.drone.move_up(30)
            elif key == ord('j'):
                self.drone.move_down(30)

    def destroy_node(self):
        self.get_logger().info('Shutting down node...')
        self.drone.streamoff()
        self.drone.end()
        super().destroy_node()

def euler_to_quaternion(r):
        """
        https://github.com/tentone/tello-ros2/blob/main/workspace/src/tello/tello/node.py
        """
        (yaw, pitch, roll) = (r[0], r[1], r[2])
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = TelloPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

