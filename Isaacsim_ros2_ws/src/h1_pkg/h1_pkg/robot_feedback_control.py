import rclpy 
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from h1_msgs.msg import ProtoOdom

class RobotFbControl(Node):
    def __init__(self):
        super().__init__('robot_rb_control_node')
        
        # Declare parmeter
        self.declare_parameter('cmd', '0 0')
        self.cmd = [float(i) for i in self.get_parameter('cmd').get_parameter_value().string_value.split()]
        
        self.lin_v_kp = 0.5
        self.ang_p_kp = 0.02
        self.max_lin_vel = 0
        self.max_ang_vel = 2.0
        
        self.create_subscription(ProtoOdom, '/proto_odom', self.callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info(f'PID: [{self.lin_v_kp, self.ang_p_kp}] -> {self.cmd}')
        
    
    def callback(self, msg):
        # Tune Angular Velocity
        
        # Linear Error
        # lin_err = self.cmd[0] - msg.vel.x
        
        # Angular Error
        ang_error = self.cmd[1] - msg.yaw
        _sign = 1 if ang_error >= 0 else -1
        
        cmd = Twist()
        # cmd.linear.x = lin_err * self.lin_v_kp
        cmd.linear.x = self.cmd[0]
        cmd.angular.z = min(abs(ang_error * self.ang_p_kp), self.max_ang_vel) * _sign
        
        # print(f'{cmd.linear.x:.3f}, {cmd.angular.z:.3f}')
        print(ang_error)
        self.cmd_pub.publish(cmd)
        
        
def main(args = None):
    rclpy.init(args= args)
    
    robot_rb_control = RobotFbControl()
    rclpy.spin(robot_rb_control)
    
    robot_rb_control.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
    
        