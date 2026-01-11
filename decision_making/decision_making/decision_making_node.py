import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class DecisionMaker(Node):
    def __init__(self):
        super().__init__('decision_maker')

        self.current_error = 0
        self.junction_status = 0

        # Sub
        self.error_sub = self.create_subscription(
            Int32,
            '/error',
            self.error_callback,
            10
        )
        
        self.junction_sub = self.create_subscription(
            Int32,
            '/junction_detected',
            self.junction_callback,
            10
        )

        self.timer = self.create_timer(0.2, self.decision_logic)

        self.get_logger().info("Decision Maker Node has started!")

    def error_callback(self, msg):
        self.current_error = msg.data

    def junction_callback(self, msg):
        self.junction_status = msg.data

    def decision_logic(self):
        """
        Logic func
        """
        direction = "STRAIGHT"
        if self.current_error > 20:
            direction = "TURN RIGHT"
        elif self.current_error < -20:
            direction = "TURN LEFT"

        junction_str = "JUNCTION DETECTED!" if self.junction_status == 1 else "Normal Road"

        self.get_logger().info(
            f"[{direction}] | Error: {self.current_error:4d} | Status: {junction_str}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = DecisionMaker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()