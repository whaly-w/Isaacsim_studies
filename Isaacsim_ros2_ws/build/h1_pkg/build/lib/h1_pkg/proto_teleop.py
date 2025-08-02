import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class ProtoTeleop(Node):
    def __init__(self):
        super().__init__('proto_teleop_node')
        
        # Declare parmeter
        self.declare_parameter('cmd', '0 0')
        
        # Retreive parameter
        self.cmd = self.get_parameter('cmd').get_parameter_value().string_value
        
        # Setup publisher
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, self.callback)
        
        
    def callback(self):
        msg = Twist()
        val = [float(i) for i in self.cmd.split()]
        msg.linear.x = val[0]
        msg.angular.z = val[1]
        
        print(self.cmd)
        self.pub.publish(msg)

def main(args= None):
    rclpy.init(args= args)
    
    proto_teleop = ProtoTeleop()
    rclpy.spin(proto_teleop)
    
    proto_teleop.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
    

# cmd => ros2 run h1_pkg proto_teleop --ros-args -p cmd:="1 2"

