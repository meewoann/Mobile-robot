#====================================================Camera====================================================
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import time
import numpy as np
from cv_bridge import CvBridge
import os

class CameraReaderNode(Node):
    def __init__(self):
        super().__init__('camera_reader_node')
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(0.033, self.timer_callback)  # Publish at 10 Hz
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture('src/camera_reader/camera_reader/test1.mp4')  # Open the default camera
        self.prev_time = 0.0
        self.cur_time = 0.0
        print("CameraReaderNode Process ID: ", os.getpid())

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
            rclpy.shutdown()

    def timer_callback(self):
        self.cur_time = time.time()
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)
        else:
            self.get_logger().warning("Failed to capture frame from camera")
        # print("Camera frame rate: {:.2f} FPS".format(1.0 / (self.cur_time - self.prev_time)))
        # print("Camera timer callback: {:.2f} ms".format((self.cur_time - self.prev_time) * 1e3))
        self.prev_time = self.cur_time

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
