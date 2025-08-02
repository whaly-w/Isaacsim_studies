import rclpy
from rclpy.node import Node

import sys
from custom_interface.srv import TestSrv

class TestServiceClient(Node):
    def __init__(self):
        super().__init__('TestServiceClientNode')
        
        self.cli = self.create_client(TestSrv, '/custom_interface/test_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TestSrv.Request()
        
    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.get_logger().info('Triggered in client')
        return self.cli.call_async(self.req)
    
def main(args= None):
    rclpy.init(args= args)
    
    test_service_client_node = TestServiceClient()
    a, b = int(sys.argv[1]), int(sys.argv[2])
    print(a, b)
    future = test_service_client_node.send_request(a, b)
    
    rclpy.spin_until_future_complete(test_service_client_node, future)
    test_service_client_node.get_logger().info(f'Respond: {future.result()}')

    test_service_client_node.destroy_node()
    rclpy.shutdown()
