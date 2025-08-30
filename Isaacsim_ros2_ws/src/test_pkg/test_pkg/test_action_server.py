import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from custom_interface.action import TestAction
from time import sleep as delay

class TestActionServer(Node):
    def __init__(self):
        super().__init__('test_action_server_node')
        
        self._action_server = ActionServer(
            self,
            TestAction,
            '/test_action',
            self.callback
        )
        
    def callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = TestAction.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i-1] + feedback_msg.partial_sequence[i]
            )
            self.get_logger().info(f'Feedback >> {feedback_msg.partial_sequence}')
            
            goal_handle.publish_feedback(feedback_msg)
            # delay(0.1)
            
        goal_handle.succeed()

        result = TestAction.Result()
        result.sequence = feedback_msg.partial_sequence
        return result
        

def main(args=None):
    rclpy.init(args=args)

    test_action_server = TestActionServer()

    rclpy.spin(test_action_server)


if __name__ == '__main__':
    main()