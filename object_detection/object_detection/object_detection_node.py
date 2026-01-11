import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point  # <-- Thêm mới
from cv_bridge import CvBridge
import cv2
import numpy as np

class MultiColorBallTracker(Node):
    def __init__(self):
        super().__init__('multi_color_ball_tracker')

        self.publisher_ = self.create_publisher(Image, 'detected_balls', 10)
        
        # Publisher tọa độ tâm 
        self.pos_pub = self.create_publisher(Point, 'ball_position', 10) 
        
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
            
        self.bridge = CvBridge()
        self.min_area = 500

        # HSV range
        self.colors = {
            'GREEN':  {'lower': np.array([35, 100, 40]), 'upper': np.array([85, 255, 255]), 'draw': (0, 255, 0)},
            'YELLOW': {'lower': np.array([20, 100, 100]), 'upper': np.array([30, 255, 255]), 'draw': (0, 255, 255)},
            'RED1':   {'lower': np.array([0, 100, 100]),  'upper': np.array([10, 255, 255]), 'draw': (0, 0, 255)}
            # 'RED2':   {'lower': np.array([160, 100, 100]), 'upper': np.array([180, 255, 255]), 'draw': (0, 0, 255)}
        }

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        global_largest_contour = None
        global_max_area = 0
        best_color_name = ""
        best_draw_color = (0, 0, 0)

        for color_name, config in self.colors.items():
            mask = cv2.inRange(hsv, config['lower'], config['upper'])
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.dilate(mask, kernel, iterations=5)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                local_largest = max(contours, key=cv2.contourArea)
                local_area = cv2.contourArea(local_largest)

                if local_area > global_max_area:
                    global_max_area = local_area
                    global_largest_contour = local_largest
                    best_color_name = color_name
                    best_draw_color = config['draw']

        if global_max_area > self.min_area:
            (x, y), r = cv2.minEnclosingCircle(global_largest_contour)
            x, y, r = int(x), int(y), int(r)

            # Tọa độ tương đối so với tâm camera
            x_rel = x - cx
            y_rel = cy - y # y hướng lên trên là dương

            # --- PUBLISH TỌA ĐỘ ---
            point_msg = Point()
            point_msg.x = float(x_rel)
            point_msg.y = float(y_rel)
            point_msg.z = float(r) # bán kính (r)
            self.pos_pub.publish(point_msg)
            # ----------------------

            cv2.circle(frame, (x, y), r, best_draw_color, 2)
            cv2.circle(frame, (x, y), 3, (0, 0, 255), -1) 
            
            display_text = f"{best_color_name} | x:{x_rel} y:{y_rel} r:{r}"
            cv2.putText(frame, display_text, (x - 50, y - r - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, best_draw_color, 2)

        # Vẽ tâm camera
        cv2.line(frame, (cx - 10, cy), (cx + 10, cy), (255, 0, 0), 1)
        cv2.line(frame, (cx, cy - 10), (cx, cy + 10), (255, 0, 0), 1)

        # Publish ảnh debug
        self.publisher_.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MultiColorBallTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()