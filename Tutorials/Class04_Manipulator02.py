# Objectives
# 1) Introduce concept of multiple robot in a simulation
# 2) Introduce how to effectively build program logic to switch between subtasks


from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers import PickPlaceController
from isaacsim.robot.manipulators.examples.franka.tasks import PickPlace

 
# Setups
world = World()
world.scene.add_default_ground_plane()

# Using task to add everyting
world.add_task(PickPlace(name='super_cool_task'))

# Reset world
world.reset()

# Setup robot and cube from task
task_params = world.get_task('super_cool_task').get_params()
franka = world.scene.get_object(task_params['robot_name']['value'])
cool_cube_name = task_params['cube_name']['value']

# Create controller
my_controller = PickPlaceController(
    name= 'pick_place_controller',
    gripper= franka.gripper,
    robot_articulation= franka
)


# Infinite Loop
while True:
    if my_controller.is_done():
        world.pause()
    
    current_observations = world.get_observations()
    
    # apply action
    actions = my_controller.forward(
        picking_position= current_observations[cool_cube_name]['position'],
        placing_position= current_observations[cool_cube_name]['target_position'],
        current_joint_positions= current_observations[franka.name]['joint_positions']
    )
    franka.apply_action(actions)
    
    world.step(render= True)
    
    

simulation_app.close()



