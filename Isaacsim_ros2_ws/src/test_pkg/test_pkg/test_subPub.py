import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubPub(Node):
    def __init__(self):
        super().__init__('minimal_subPub')
        self.subscription = self.create_subscription( String, 'topic', self.listener_callback, 10)
        self.pub = self.create_publisher(String, '/new_topic', 10)
        # self.create_timer(1, self.publish_callback)
        
    def listener_callback(self, msg):
        new_msg = String()
        old_msg = msg.data.split(': ')
        new_msg.data = f'{old_msg[0]}: {int(old_msg[-1])*2} - edited by Whaly'
        self.pub.publish(new_msg)
        
        


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubPub()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()