# Objectives
# 1) Introduce a new type of robot and controller


from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers import PickPlaceController

 
# Setups
world = World()
world.scene.add_default_ground_plane()

# Using Franka extension to add the robot
franka = world.scene.add(Franka(prim_path= '/World/Fancy_Franka', name= 'my_franka'))

random_cube = world.scene.add(DynamicCuboid(
    prim_path= '/World/random_cube',
    name= 'random_cube',
    position= np.array([0.3, 0.3, 0.3]),
    scale= np.array([0.0515, 0.0515, 0.0515]),
    color= np.array([0, 0, 1.0])
))

# Create controller
my_controller = PickPlaceController(
    name= 'pick_place_controller',
    gripper= franka.gripper,
    robot_articulation= franka
)

# Reset world
world.reset()

# Reset position of franka gripper
franka.gripper.set_joint_positions(franka.gripper.joint_opened_positions)


# Infinite Loop
while True:
    
    if my_controller.is_done():
        world.pause()
    
    # pos, orien
    cube_position, _ = random_cube.get_world_pose() 
    goal_position = np.array([-0.3, -0.3, 0.0515 / 2.0])
    current_joint_positions = franka.get_joint_positions()
    
    # apply action
    actions = my_controller.forward(
        picking_position= cube_position,
        placing_position= goal_position,
        current_joint_positions= current_joint_positions
    )
    franka.apply_action(actions)
    
    world.step(render= True)
    
    

simulation_app.close()



