# Objectives
# 1) Dealing with tilted IMU 
# 2) Set render and physics rate


from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

# Custom Package
from PhysicsStruct import *

import numpy as np
from isaacsim.core.api import World

from isaacsim.sensors.physics import IMUSensor
from isaacsim.core.api.objects import DynamicCuboid

import omni
from omni.isaac.core.utils.rotations import euler_angles_to_quat



### Setups
world = World()
world.scene.add_default_ground_plane()
world.set_simulation_dt(physics_dt= 1.0 / 100, rendering_dt= 5.0 / 100)

# Add a cube
random_cube = world.scene.add(DynamicCuboid(
    prim_path="/World/random_cube",
    name="random_cube",
    position=np.array([0, 0, 0.25]),
    size= 0.5,
    color=np.array([0, 0, 1.0]),
    mass= 1.0,
))

# Add IMU
IMU = IMUSensor(
    prim_path= '/World/random_cube/IMU1',
    name= 'fancy_robot_imu1',
    frequency= 200,
    translation= np.array([0, 0, 0]),
    orientation= np.array([1, 0, 0, 0]),
    linear_acceleration_filter_size= 15,
    angular_velocity_filter_size= 15,
    orientation_filter_size= 15
)

tilted = [np.pi/4, np.pi/4, 0]
IMU_tilted = IMUSensor(
    prim_path= '/World/random_cube/IMU2',
    name= 'fancy_robot_imu2',
    frequency= 200,
    translation= np.array([0, 0, 0]),
    orientation= euler_angles_to_quat(tilted),
    linear_acceleration_filter_size= 15,
    angular_velocity_filter_size= 15,
    orientation_filter_size= 15
)


### Reset World
world.reset()


### Global Variables
_prev_time = 0
_lin_acc = cartesian_vector()
_lin_vel = cartesian_vector()
_target_vel = np.array([0.1, 0, 0])
_cmd = np.zeros(3)

acc = cartesian_vector()
acc_tilted = cartesian_vector()


### Infinite Loop
while True:
    random_cube.set_linear_velocity(_cmd)
    
    world.step(render= True)
    
    if world.is_stopped():
        _lin_vel.set(np.zeros(3))
        _cmd = np.zeros(3)
        continue
    
    
    value_1 = IMU.get_current_frame(read_gravity= True)
    value_2 = IMU_tilted.get_current_frame(read_gravity= True)
    
    acc.set(value_1['lin_acc'])
    acc_tilted.set(value_2['lin_acc'])
    
    print('ACC01 ->', acc.get_round())
    print('ACC02 ->', acc_tilted.get_round())
    
    acc_tilted.rotate(tilted)
    print('ACC02_tilted ->', acc_tilted.get_round())
    print('------------------------------------------')
    
    
    _cmd = _target_vel * value_1['time']

simulation_app.close()



