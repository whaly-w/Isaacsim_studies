import rclpy
from rclpy.node import Node

import sys
from custom_interface.srv import TestSrv

class TestServiceClient(Node):
    def __init__(self):
        super().__init__('TestServiceClientNode')
        
        # Create client to handle service
        self.cli = self.create_client(TestSrv, '/custom_interface/test_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            
        # Create variable with the service class for data manipulation
        self.req = TestSrv.Request()
        
        
    def send_request(self, a, b):
        # Set input data
        self.req.a = a
        self.req.b = b
        self.get_logger().info('Triggered in client')
        
        # call service, can also use .call() to make the service synchronous
        return self.cli.call_async(self.req)
    
    
def main(args= None):
    rclpy.init(args= args)
    
    # Take variable from cmd
    a, b = int(sys.argv[1]), int(sys.argv[2])
    print('Input:', a, b)
    
    test_service_client_node = TestServiceClient()
    future = test_service_client_node.send_request(a, b)
    
    # Loop until respond from service is received
    rclpy.spin_until_future_complete(test_service_client_node, future)
    test_service_client_node.get_logger().info(f'Respond >> {a} + {b} = {future.result().sum}')

    test_service_client_node.destroy_node()
    rclpy.shutdown()

# To run node, make sure you add value for a and b >> ros2 run test_pkg service_client 2 3 >> a= 2, b= 3