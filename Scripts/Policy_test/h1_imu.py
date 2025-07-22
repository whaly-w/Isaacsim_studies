from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from PhysicsStruct import *

import numpy as np
import carb 
import omni
import omni.appwindow
from isaacsim.core.api import World

from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.policy.examples.robots.h1 import H1FlatTerrainPolicy

from isaacsim.sensors.physics import IMUSensor
from omni.isaac.core.utils.rotations import quat_to_euler_angles



### Functions
def _sub_keyboard_event(event, *args, **kwargs) -> bool:
    global _input_keyboard_mapping, _base_command, _time, _time_threshold
    
    if event.type == carb.input.KeyboardEventType.KEY_PRESS and _time > _time_threshold:
        # on pressing, the command is incremented
        if event.input.name in _input_keyboard_mapping:
            if event.input.name == 'X':
                _base_command = _input_keyboard_mapping[event.input.name]
            else:
                _base_command += np.array(_input_keyboard_mapping[event.input.name])
            
    return True



### Setups
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane(
    z_position=0,
    name="default_ground_plane",
    prim_path="/World/defaultGroundPlane",
    static_friction=0.2,
    dynamic_friction=0.2,
    restitution=0.01,)

world.set_simulation_dt(physics_dt= 1.0 / 200, rendering_dt= 2.0 / 200)


# Global Variables
_time_threshold = 1.0
_movement_vel = 0.1
_input_keyboard_mapping = {
    "W": [_movement_vel, 0.0, 0.0],
    "A": [0.0, 0.0, _movement_vel],
    "S": [-_movement_vel, 0.0, 0.0],
    "D": [0.0, 0.0, -_movement_vel],
    "X": [0.0, 0.0, 0.0]}
_oren = orientation_rpy()
_lin_acc = IMU_acc()
_lin_vel = cartesian_vector()
_base_command = [0.0, 0.0, 0.0]
_physics_ready = False
_prev_t = 0


# Add H1
assets_root_path = get_assets_root_path()
h1 = H1FlatTerrainPolicy(
    prim_path="/World/H1",
    name="H1",
    usd_path=assets_root_path + "/Isaac/Robots/Unitree/H1/h1.usd",
    position=np.array([0, 0, 1.05]), )

# Add IMU
IMU = IMUSensor(
    prim_path= '/World/H1/imu_link/IMU',
    name= 'h1_imu',
    frequency= 500,
    translation= np.array([0, 0, 0]),
    orientation= np.array([1, 0, 0, 0]),
    linear_acceleration_filter_size= 35,
    angular_velocity_filter_size= 25,
    orientation_filter_size= 25
)



### Post Load
_appwindow = omni.appwindow.get_default_app_window()
_input = carb.input.acquire_input_interface()
_keyboard = _appwindow.get_keyboard()
_input.subscribe_to_keyboard_events(_keyboard, _sub_keyboard_event)



### Reset World
world.reset()



### Infinite Loop
while True:
    world.step(render= True)

    if _physics_ready and not world.is_stopped():
        h1.forward(world.get_physics_dt(), _base_command)
    elif world.is_stopped():
        _physics_ready = False
    else:
        _physics_ready = True
        h1.initialize()
        h1.post_reset()
        h1.robot.set_joints_default_state(h1.default_pos)
        _lin_vel.set((0, 0, 0))
        _base_command = np.zeros(3)
    

    value = IMU.get_current_frame(read_gravity= True)
    _time = value['time']
    if _time > _time_threshold:
        dt = _time - _prev_t
        
        _oren.set(quat_to_euler_angles(value['orientation']))
        
        _lin_acc.set(value['lin_acc'])
        _lin_acc.rotate(_oren.get(), neglect_gravity= True)
        
        _lin_vel.set(dt * _lin_acc.get(), inc= True)
        print('CMD ->', [round(i, 1) for i in _base_command])
        # print('ACC ->', _lin_acc.get_round())
        # print('VEL ->', _lin_vel.get_round())
        # print('OREN ->', _oren.get_deg(dec= 1), '\n')
        print('YAW:', _oren.get_deg(dec= 1)[2])
        _lin_vel.print()
        print('--------------------------------------')
    else:
        print('Waiting for sensor initiation...')
    _prev_t = _time
        

    

simulation_app.close()



