## Overview
This package is created as a example of creating custom message, service, and action <br>
**Noted** that you can only create these custom data using *ament_cmake*

## Instruciton
To avoid python version problem, first set python version for catkin in **CMakeLists.txt**. <br>
**This must be added before *ament_cmake***
```
set(PYTHON_EXECUTABLE "/usr/bin/python3.10")
find_package(ament_cmake REQUIRED)
```

Next add these lines to **CMakeLists.txt** to allow colcon to auto build the custom data
- Add the path in *rosidl_generate_interfaces*
- Also add the dependencies if any msg or srv requires it
```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TestMsg.msg"
  "srv/TestSrv.srv"
  "srv/TestSrvEmptyInput.srv"
  "action/TestAction.action"
  DEPENDENCIES geometry_msgs
)

```

Next add for *message* and *service*, these lines to **package.xml**
```
  <depend>geometry_msgs</depend> # add the dependency used in each msg or srv

  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```
For *action*, add these instead.
```
    <buildtool_depend>rosidl_default_generators</buildtool_depend>
    <depend>action_msgs</depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
```

## File Structure
For ROS2 message
```
# variable_type variable_name
```

For ROS2 service, input or output can be empty like in **srv/TestSrvEmptyInput.srv**
```
# Input variables
---
# Output variables
```

For ROS2 action
```
# Request
---
# Result
---
# Feedbace
```
