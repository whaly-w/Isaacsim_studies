import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/whaly/isaac_ws/Isaacsim_ros2_ws/install/ackermann_control'
