import rclpy
from rclpy.node import Node

from custom_interface.srv import TestSrv
from custom_interface.srv import TestSrvEmptyInput

class TestService(Node):
    def __init__(self):
        super().__init__('TestServiceNodeHost')
        # Create two services
        self.create_service (TestSrv, '/custom_interface/test_service', self.service_callback01)
        self.create_service (TestSrvEmptyInput, '/custom_interface/test_service_no_input', self.service_callback02)
    
    def service_callback01(self, req, res):
        # set the value of output
        res.sum = req.a + req.b
        
        # log the input value
        self.get_logger().info(f'Service Triggered >> [{req.a}, {req.b}]')
        return res
    
    def service_callback02(self, req, res):
        res.result = True
        self.get_logger().info(f'Service Triggered >> True')
        return res


def main(args= None):
    rclpy.init(args= args)
    
    test_service = TestService()
    rclpy.spin(test_service)
    
    test_service.destroy_node()
    rclpy.shutdown()
        
        