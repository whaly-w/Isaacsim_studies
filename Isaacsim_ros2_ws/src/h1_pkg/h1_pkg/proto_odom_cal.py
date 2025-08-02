import rclpy
from rclpy.node import Node

import numpy as np
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Vector3
from h1_msgs.msg import ProtoOdom

# from PhysicsStruct import *
from .VectorSpace3D import *

class OdomCal(Node):
    def __init__(self):
        super().__init__('odom_cal_node')
        
        # Define parameters
        self.vel_thresh = 0.001
        
        # Define global variables
        self._lin_acc = IMU_acc()
        self._oren = orientation_rpy()
        self._lin_vel = cartesian_vector()
        self._ang_vel = cartesian_vector()
        self._time = 0
        self._prev_time = 0
        
        # Create subscriptions
        self.imu_sub = self.create_subscription(Imu, "/imu_data", self.callback, 10)
        
        self._time_threshold = None
        self.threshhold_sub = self.create_subscription(Float32, '/time_threshold', self.threshold_callback, 10)
        
        # Create publisher
        self.proto_odom_pub = self.create_publisher(ProtoOdom, '/proto_odom', 10)
         
        
    def threshold_callback(self, msg):
        self._time_threshold = msg.data
    
    
    def callback(self, msg):
        self._time = msg.header.stamp.sec + float(f'0.{msg.header.stamp.nanosec:09d}'[:4])
        if self._time_threshold is None:
            return
        
        if self._time > self._time_threshold:
            dt = self._time - self._prev_time
            
            self._oren.set_ros2_geometry_msgs_quaternion(msg.orientation)
            
            self._ang_vel.set_ros2_geometry_msgs_vector3(msg.angular_velocity)
            self._ang_vel.rotate(np.array([self._oren.get()[0], self._oren.get()[1], 0]))
            
            self._lin_acc.set_ros2_geometry_msgs_vector3(msg.linear_acceleration)
            self._lin_acc.rotate(self._oren.get(), neglect_gravity= True)
            
            dv = dt * self._lin_acc.get()
            # self._lin_vel.set(dv, inc= True)
            self._lin_vel.set(self.zero_velocity_filter(dv), inc= True)
            # self.zero_velocity_filter(self._lin_vel.get())
            print(dv)
            print('YAW:', self._oren.get_deg(dec= 1)[2])
            print(f'lin_vel.x: {self._lin_vel.x:.3f}')
            print(f'ang_vel.z: {self._ang_vel.z:.3f}')
            # self._lin_vel.print()
            print('--------------------------------------')
            
            # Publish calculated odom
            proto_odom = ProtoOdom()
            proto_odom.vel.x = self._lin_vel.x
            proto_odom.vel.y = self._lin_vel.y
            proto_odom.vel.z = self._ang_vel.z
            proto_odom.yaw = self._oren.get_deg(dec= 3)[2]
            
            self.proto_odom_pub.publish(proto_odom)
            
        else:
            print(f'Waiting for sensor initiation... [{self._time}]s')
            self._lin_vel.set((0, 0, 0))
        self._prev_time = self._time
    
    def zero_velocity_filter(self, dv):
        for i, vel in enumerate(dv):
            if abs(vel) < self.vel_thresh:
                dv[i] = 0
        return dv
            
        
            
        

 
        
def main(args= None):
    rclpy.init(args= args)
    
    odom_cal = OdomCal()
    rclpy.spin(odom_cal)
    
    odom_cal.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        