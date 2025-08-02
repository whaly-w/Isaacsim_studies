import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        
        self.declare_parameter('sub_topic', '/topic')
        self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(
            String,
            self.sub_topic,
            self.listener_callback,
            10)
        

    def listener_callback(self, msg):
        self.get_logger().info(f'Sub to \"{self.sub_topic}\" -> msg: \"{msg.data}\"')
        # self.get_logger().info('I heard: "%s"' % msg.data)
        # print(f'I heard \"{msg.data}\"')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()