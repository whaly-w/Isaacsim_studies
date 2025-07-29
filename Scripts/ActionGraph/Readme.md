## Instruction
To test **InteractionWithGraph.py**, use the following command to publish /cmd_vel data
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
  x: 10.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```