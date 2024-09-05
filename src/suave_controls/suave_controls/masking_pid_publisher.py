import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3

class MaskingPIDPublisher(Node):

    def __init__(self):
        super().__init__('masking_pid_publisher')
        self.publisher_ = self.create_publisher(Vector3, 'masking_pid_publisher', 10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Vector3()
        msg.x = float(1)
        msg.y = float(2)
        msg.z = float(3)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing masking pid publisher!')

def main(args=None):
    print("[main] suave_controls/masking_pid_publisher.py")

    rclpy.init(args=args)

    maskingPIDPublisher = MaskingPIDPublisher()

    rclpy.spin(maskingPIDPublisher)

    maskingPIDPublisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
