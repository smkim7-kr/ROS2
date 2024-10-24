import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CameraInfoPrinter(Node):
    def __init__(self):
        super().__init__('camera_info_printer')
        # Subscribe to the /camera_info topic
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.camera_info_callback,
            10
        )

    def camera_info_callback(self, msg):
        # Print the metadata from CameraInfo message
        self.get_logger().info('Received Camera Info:')
        self.get_logger().info(f"Width: {msg.width}, Height: {msg.height}")
        self.get_logger().info(f"Distortion Model: {msg.distortion_model}")
        self.get_logger().info(f"Focal Lengths (fx, fy): ({msg.k[0]}, {msg.k[4]})")
        self.get_logger().info(f"Principal Point (cx, cy): ({msg.k[2]}, {msg.k[5]})")
        self.get_logger().info(f"Distortion Coefficients: {msg.d}")
        self.get_logger().info(f"Camera Matrix: {msg.k}")
        self.get_logger().info(f"Projection Matrix: {msg.p}")

def main(args=None):
    rclpy.init(args=args)
    camera_info_printer = CameraInfoPrinter()

    try:
        rclpy.spin(camera_info_printer)
    except KeyboardInterrupt:
        pass
    finally:
        camera_info_printer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
