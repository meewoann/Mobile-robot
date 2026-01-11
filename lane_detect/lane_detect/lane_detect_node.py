#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from cv_bridge import CvBridge

import cv2
import numpy as np
from ultralytics import YOLO

# ================= CONFIG =================
MODEL_PATH = r"/home/meewoan/Comvi_ws/src/lane_detect/runs_segver3/lane_segver3/weights/best.pt"

FRAME_W = 640
FRAME_H = 480
SCAN_RATIO = 0.8
SCAN_ROW = int(FRAME_H * SCAN_RATIO)

CONF = 0.8
DEVICE = "cpu"    

JUNCTION_WIDTH_THRESH = 350   # pixel, tự chỉnh theo camera

# =========================================


def compute_lane_center(mask, scan_row, last_center):
    row = mask[scan_row, :]
    xs = np.where(row == 255)[0]

    if len(xs) > 0:
        left = xs[0]
        right = xs[-1]
        center = (left + right) // 2
        return center, left, right
    else:
        return last_center, None, None


class YoloLaneNode(Node):
    def __init__(self):
        super().__init__("yolo_lane_node")

        self.bridge = CvBridge()
        self.model = YOLO(MODEL_PATH)

        self.prev_mask = None
        self.lane_memory = None

        self.image_center_x = FRAME_W // 2

        self.sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10
        )

        self.junction_pub = self.create_publisher(
            Int32,
            "/junction_detected",
            10
        )

        self.error_pub = self.create_publisher(
            Int32,
            "/error",
            10
        )

        self.lane_pub = self.create_publisher(
            Image,
            "/camera/lane",
            10
        )


        self.get_logger().info("YOLO Lane Node started")

    def image_callback(self, msg: Image):
        # ===== ROS Image -> OpenCV =====
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        frame = cv2.resize(frame, (FRAME_W, FRAME_H))

        annotated = frame.copy()
        current_mask = None

        # ===== YOLO SEG =====
        results = self.model(
            frame,
            conf=CONF,
            imgsz=640,
            device=DEVICE,
            verbose=False
        )

        # ===== MASK =====
        if results[0].masks is not None:
            masks = results[0].masks.data.cpu().numpy()

            combined_mask = np.zeros(masks[0].shape, dtype=np.uint8)
            for m in masks:
                combined_mask = np.logical_or(combined_mask, m > 0.5)

            current_mask = combined_mask.astype(np.uint8) * 255
            self.prev_mask = current_mask
        else:
            current_mask = self.prev_mask

        # ===== DRAW =====
        if current_mask is not None:
            overlay = np.zeros_like(annotated)
            overlay[:, :, 1] = 255

            annotated = np.where(
                current_mask[:, :, None] == 255,
                annotated * 0.5 + overlay * 0.5,
                annotated
            ).astype(np.uint8)

            lane_center, left, right = compute_lane_center(
                current_mask,
                SCAN_ROW,
                self.lane_memory
            )

            # ===== SCAN LINE =====
            cv2.line(
                annotated,
                (0, SCAN_ROW),
                (FRAME_W, SCAN_ROW),
                (255, 255, 0),
                1
            )

            # ===== IMAGE CENTER =====
            cv2.circle(
                annotated,
                (self.image_center_x, SCAN_ROW),
                8,
                (255, 0, 0),
                -1
            )

            if lane_center is not None:
                self.lane_memory = lane_center

                if left is not None and right is not None:
                    cv2.circle(annotated, (left, SCAN_ROW), 5, (0, 255, 0), -1)
                    cv2.circle(annotated, (right, SCAN_ROW), 5, (0, 255, 0), -1)

                cv2.circle(
                    annotated,
                    (lane_center, SCAN_ROW),
                    8,
                    (0, 0, 255),
                    -1
                )

                error = lane_center - self.image_center_x

                junction_msg = Int32()
                junction_msg.data = 0
                
                error_msg = Int32()
                error_msg.data = int(error)
                self.error_pub.publish(error_msg)

                if left is not None and right is not None:
                    lane_width = right - left

                    if lane_width > JUNCTION_WIDTH_THRESH:
                        junction_msg.data = 1

                        cv2.putText(
                            annotated,
                            "JUNCTION DETECTED",
                            (20, 110),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            (0, 0, 255),
                            2
                        )

                self.junction_pub.publish(junction_msg)


                ros_image = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                self.lane_pub.publish(ros_image)

                cv2.putText(
                    annotated,
                    f"Lane Center: {lane_center}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2
                )

                cv2.putText(
                    annotated,
                    f"Error: {error}",
                    (20, 75),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 0, 0),
                    2
                )

                ros_image = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                self.lane_pub.publish(ros_image)

        # cv2.imshow("YOLO Lane + Center Debug", annotated)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = YoloLaneNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
