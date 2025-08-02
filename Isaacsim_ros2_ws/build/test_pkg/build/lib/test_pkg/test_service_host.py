import rclpy
from rclpy.node import Node

from custom_interface.srv import TestSrv

class TestService(Node):
    def __init__(self):
        super().__init__('TestServiceNodeHost')
        self.create_service (TestSrv, '/custom_interface/test_service', self.service_callback)
        
    def service_callback(self, req, res):
        res.sum = req.a + req.b
        self.get_logger().info(f'Service Triggered >> [{req.a}, {req.b}]')
        return res

def main(args= None):
    rclpy.init(args= args)
    
    test_service = TestService()
    rclpy.spin(test_service)
    
    test_service.destroy_node()
    rclpy.shutdown()
        
        