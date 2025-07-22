# Objectives
# 1) Add robot using WheeledRobot extension
# 2) Add a controller to command the robot


from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

import numpy as np
from isaacsim.core.api import World
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.types import ArticulationAction
import carb

from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.core.api.controllers import BaseController
from isaacsim.robot.wheeled_robots.controllers.wheel_base_pose_controller import WheelBasePoseController
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController

 

# Create a custom controller class
class CoolController(BaseController):
    def __init__(self):
        super().__init__(name="my_cool_controller")

        # Setup robot dimensions
        self._wheel_radius = 0.03
        self._wheel_base = 0.1125
        return

    def forward(self, command):
        # command will have two elements, first element is the forward velocity
        # second element is the angular velocity (yaw only).
        joint_velocities = [0.0, 0.0]
        joint_velocities[0] = ((2 * command[0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[1] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)

        # A controller has to return an ArticulationAction
        return ArticulationAction(joint_velocities= joint_velocities)


# Setups
world = World()
world.scene.add_default_ground_plane()

# Configure a new path to the /Isaac folder
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find /Isaac folder")
asset_path = assets_root_path + '/Isaac/Robots/Jetbot/jetbot.usd'

# Using Wheeled Robot automatically add Robot Prim and Controller
# This creates 3 robots
jetbot = [world.scene.add(
    WheeledRobot(
        prim_path= f"/World/Fancy_Robot_{i}",
        name= f"fancy_robot_{i}",
        position= [0, 2*i, 0.2],
        wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
        create_robot= True,
        usd_path=asset_path,
        )
    ) for i in range(3)]

# Create the custom controller
my_controller_1 = CoolController()
my_controller_2 = WheelBasePoseController(name="super_cool_controller",
                                        open_loop_wheel_controller= DifferentialController(name="simple_control",
                                                                                           wheel_radius=0.03, wheel_base=0.1125),
                                        is_holonomic=False)

# Reset World
world.reset()


# Infinite Loop
while True:
    # Direct controll
    jetbot[0].apply_wheel_actions(ArticulationAction(joint_positions=None,
                                                  joint_efforts=None,
                                                  joint_velocities=5 * np.random.rand(2,)))
    # Controller 1 - set velocity
    jetbot[1].apply_action(my_controller_1.forward([0.2, np.pi/4]))
    
    # Controller 2 - set target
    position, orientation = jetbot[2].get_world_pose()
    jetbot[2].apply_action(my_controller_2.forward(start_position=position,
                                                start_orientation=orientation,
                                                goal_position=np.array([0.8, 0.8])))
    
    # This must be called after calling actions and before accessing articulate data to get the lastest update 
    world.step(render= True)
    

simulation_app.close()



