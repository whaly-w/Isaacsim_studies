import rclpy
from rclpy.node import Node

from std_msgs.msg import String
# from custom_msgs.msg import ProtoOdom
from h1_msgs.msg import ProtoOdom


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('msg_publisher')
        self.publisher_ = self.create_publisher(ProtoOdom, '/test_msg', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = ProtoOdom()
        msg.yaw = 23.0
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()