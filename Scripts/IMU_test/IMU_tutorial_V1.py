# Objectives
# 1) Create IMU sensors
# 2) Calculate linear velocity and linear displacement


from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

import numpy as np
from isaacsim.core.api import World
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api.robots import Robot

from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController

from isaacsim.sensors.physics import IMUSensor
from isaacsim.core.api.objects import DynamicCuboid



### Setups
world = World()
world.scene.add_default_ground_plane()


# Configure a new path to the /Isaac folder
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    print('Error Path NOT FOUND')
asset_path = assets_root_path + '/Isaac/Robots/Jetbot/jetbot.usd'

# Add Jetbot
add_reference_to_stage(usd_path= asset_path, prim_path= '/World/Fancy_Robot')    
jetbot = world.scene.add(Robot(prim_path= '/World/Fancy_Robot', orientation= [1, 0, 0, 0], name= 'fancy_robot'))
controller = DifferentialController("test_controller", 0.03, 0.1125)

# Add IMU
IMU = IMUSensor(
    prim_path= '/World/Fancy_Robot/chassis/IMU',
    name= 'fancy_robot_imu',
    frequency= 60,
    position= np.array([0, 0, 0]),
    # translation= np.array([0, 0, 0]),
    orientation= np.array([1, 0, 0, 0]),
    linear_acceleration_filter_size= 5,
    angular_velocity_filter_size= 5,
    orientation_filter_size= 5
)


### Reset World
world.reset()


### Global Variables
prev_t = 0
dt = 0
lin_vel = np.zeros(3)
lin_dis = np.zeros(3)


### Infinite Loop
world.pause()
while True:
    actions = controller.forward(command=[0.5, 0])
    jetbot.apply_action(actions)
    
    
    # Read IMU data
    value = IMU.get_current_frame(read_gravity= False)
    _time = value['time']
    lin_acc = value['lin_acc']
    
    dt = _time - prev_t
    prev_t = _time
    
    lin_vel += dt * lin_acc
    print(lin_acc)
    print(f'vel => x: {lin_vel[0]:.1f}\ny: {lin_vel[1]:.1f}\nz: {lin_vel[2]:.1f}')
    
    lin_dis += dt * lin_vel
    # print(f'dis => x: {lin_dis[0]:.2f}\ny: {lin_dis[1]:.2f}\nz: {lin_dis[2]:.2f}')
    
    
    abs_pos, abs_oren = jetbot.get_world_pose()
    linear_velocity = jetbot.get_linear_velocity()
    # print(abs_pos)
    print(linear_velocity, end= '\n\n')

    
    
    world.step(render= True)

    

simulation_app.close()



