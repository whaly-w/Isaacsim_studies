import rclpy 
from rclpy.node import Node
from rclpy.action import ActionServer

from custom_interface.action import TurtleDrawAction
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from time import sleep as delay
from copy import deepcopy


class TurtleDrawActionServer(Node):
    def __init__(self):
        super().__init__('TurtleDrawActionServerNode')
        
        # Set global variable
        self._turtle_pose = None
        self._state = 0
        
        # Create action
        self._action_sever = ActionServer(
            self, 
            TurtleDrawAction, 
            '/turtle_draw_square', 
            self.callback_draw_square)
        
        # Create sub & pub
        self._pub_cmd_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self._sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.sub_pose_callback, 10)


    async def callback_draw_square(self, goal_handle):
        size = goal_handle.request.size
        feedback_msg = TurtleDrawAction.Feedback()
        self.get_logger().info('Begin drawing with side = {size}')
        
        initial_pose = Pose()
        while self._turtle_pose is None:
            delay(0.01)
        initial_pose = deepcopy(self._turtle_pose)
        # print(initial_pose)
        
        while True:
            print(initial_pose.x, self._turtle_pose.x)
            # print(self._turtle_pose)
            # State logic
            if self._turtle_pose.x - initial_pose.x >= size and self._state == 0:
                self._state = 1
            elif self._turtle_pose.theta - initial_pose.theta >= 1.57 and self._state == 1:
                self._state = 2
            elif self._turtle_pose.y - initial_pose.y >= size and self._state == 2:
                self._state = 3
            elif self._turtle_pose.theta - initial_pose.theta >= 3.14 and self._state == 3:
                self._state = 4
            elif self._turtle_pose.x - initial_pose.x <= 0 and self._state == 4:
                self._state = 5
            elif abs(self._turtle_pose.theta - initial_pose.theta) <= 1.57 and self._state == 5:
                self._state = 6
            elif self._turtle_pose.y - initial_pose.y <= 0 and self._state == 6:
                self._state = 7
            elif self._turtle_pose.theta - initial_pose.theta >= 0 and self._state == 7:
                self._state = 8
                break
            
            # CMD
            cmd = Twist()
            if self._state in [0, 2, 4, 6]:
                cmd.linear.x = 1.0
            elif self._state in [1, 3, 5, 7]:
                cmd.angular.z = 1.0
            self._pub_cmd_vel.publish(cmd)
            
            # print(self._turtle_pose.x - initial_pose.x)
            
            # feedback_msg.state = self._state
            # goal_handle.publish_feedback(feedback_msg)
        
        goal_handle.succeed()

        result = TurtleDrawAction.Result()
        result.isSuccess = True
        return result
            
    
    def sub_pose_callback(self, msg):
        print(msg)
        self._turtle_pose = msg
        

def main(args= None):
    rclpy.init(args= args)
    
    turtlesim_draw_action = TurtleDrawActionServer()
    rclpy.spin(turtlesim_draw_action)
    
    turtlesim_draw_action.destroy_node()
    rclpy.shutdown()  
    
    
if __name__ == '__main__':
    main()
        