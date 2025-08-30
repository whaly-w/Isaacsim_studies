import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import sys
from custom_interface.action import TestAction

class TestActionClient(Node):
    def __init__(self):
        super().__init__('test_action_client_node')
        self._action_client = ActionClient(self, TestAction, '/test_action')
        
        
    def send_goal(self, order):
        goal_msg = TestAction.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback= self.feedback_callback)
        self._send_goal_future.add_done_callback(self.callback)
        
        
    def callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('GOAL REJECTED')
            return

        self.get_logger().info('GOAL ACCEPTED')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        

    def get_result_callback(self, future):
        self.get_logger().info(f'Result >> {future.result().result.sequence}')
        rclpy.shutdown()
        
        
    def feedback_callback(self, feedback_msg):
        partial_seq = feedback_msg.feedback.partial_sequence
        self.get_logger().info(f'Feedback [{len(partial_seq)}] >> {partial_seq}')
        

def main(args=None):
    rclpy.init(args=args)   

    test_action_action = TestActionClient()
    
    test_action_action.send_goal(int(sys.argv[1]))
    
    rclpy.spin(test_action_action)
    


if __name__ == '__main__':
    main()