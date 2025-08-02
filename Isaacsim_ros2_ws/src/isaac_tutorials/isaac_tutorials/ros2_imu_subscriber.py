import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import time


class Ros2ImuSub(Node):
    def __init__(self):
        super().__init__("imu_sub_node")

        self.sub = self.create_subscription(Imu, "/imu_data", self.callback, 10)
        self._time = 0
        self._prev_time = 0


    def callback(self, msg):
        oren = msg.orientation
        lin_acc = msg.linear_acceleration
        
        print(oren)
        print(lin_acc)
        print('---------------------------------')
        


def main(args=None):
    rclpy.init(args=args)

    ros2_imu_sub = Ros2ImuSub()
    rclpy.spin(ros2_imu_sub)

    ros2_imu_sub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
