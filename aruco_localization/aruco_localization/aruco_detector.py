import rclpy
from rclpy.node import Node

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.get_logger().info('ArUco detector node started')

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
