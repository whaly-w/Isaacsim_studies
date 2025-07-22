# Objectives
# 1) Create robot
# 2) Directly set robot articulation


from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

import numpy as np
from isaacsim.core.api import World
from isaacsim.storage.native import get_assets_root_path
# from isaacsim.core.utils.nucleus import get_assets_root_path # this one is deprecated
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.types import ArticulationAction
import carb 

# Setups
world = World()
world.scene.add_default_ground_plane()

# Configure a new path to the /Isaac folder
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find /Isaac folder")
asset_path = assets_root_path + '/Isaac/Robots/Jetbot/jetbot.usd'

# This creates a container (Robot Prim) storing the robot information stored and add it to the world => /World/Fancy_Robot 
# this is a standard way to import robot, it could be use with custom robot
add_reference_to_stage(usd_path= asset_path, prim_path= '/World/Fancy_Robot')    

# Create a pointer to tag a reference to the robot from the scene
# > Articulation related information cannot be access yet as physics has not been initiated (initiate after the reset)
jetbot = world.scene.add(Robot(prim_path= '/World/Fancy_Robot', name= 'fancy_robot'))
print(f'DOF before first reset: {jetbot.num_dof}')

# Add controller to the robot
jetbot_controller = jetbot.get_articulation_controller()

# Reset World
world.reset()


# Infinite Loop
while True:
    # Actions
    jetbot_controller.apply_action(ArticulationAction(joint_positions= None,
                                                      joint_efforts= None,
                                                      joint_velocities= 5* np.random.rand(2)))
    
    # This must be called after calling actions and before accessing articulate data to get the lastest update 
    world.step(render= True)
    print(f'DOF after the reset: {jetbot.num_dof}')
    print(f'Joint Position: {jetbot.get_joint_positions()} \n')
    

simulation_app.close()



