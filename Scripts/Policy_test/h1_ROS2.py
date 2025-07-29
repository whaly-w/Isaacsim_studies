from isaacsim import SimulationApp
CONFIG = {"renderer": "RayTracedLighting", "headless": False}
simulation_app = SimulationApp(CONFIG)

from PhysicsStruct import *

import numpy as np
import omni
from isaacsim.core.api import World

from PhysicsStruct import *

from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.policy.examples.robots.h1 import H1FlatTerrainPolicy
from isaacsim.sensors.physics import IMUSensor
from omni.isaac.core.utils.rotations import quat_to_euler_angles, euler_angles_to_quat

from isaacsim.core.utils import extensions
import omni.graph.core as og

# Enable ROS2 bridge extension
extensions.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()
world = World(stage_units_in_meters=1.0)

######################################## World Setups
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane(
    z_position=0,
    name="default_ground_plane",
    prim_path="/World/defaultGroundPlane",
    static_friction=0.2,
    dynamic_friction=0.2,
    restitution=0.01,)

world.set_simulation_dt(physics_dt= 1.0 / 200, rendering_dt= 2.0 / 200)



######################################## Functions
ros_graph_path = '/Graph/ROS2Graph'
def ros2_setup(imu):
    global ros_graph_path, _time_threshold
    try:
        (ros_camera_graph, _, _, _) = og.Controller.edit(
            {
                "graph_path": ros_graph_path,
                "evaluator_name": "execution",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("RosContext", "isaacsim.ros2.bridge.ROS2Context"),
                    ("RosSubCMDVel", 'isaacsim.ros2.bridge.ROS2SubscribeTwist'),
                    
                    ("IsaacClock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                    ("PubIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
                    ('PubDelayTime', 'isaacsim.ros2.bridge.ROS2Publisher')
                ],
                og.Controller.Keys.CONNECT: [
                    ('OnPlaybackTick.outputs:tick', 'RosSubCMDVel.inputs:execIn'),
                    ('OnPlaybackTick.outputs:tick', 'ReadIMU.inputs:execIn'),
                    ('OnPlaybackTick.outputs:tick', 'PubIMU.inputs:execIn'),
                    ('OnPlaybackTick.outputs:tick', 'PubDelayTime.inputs:execIn'),
                    
                    ('IsaacClock.outputs:simulationTime', 'PubIMU.inputs:timeStamp'),
                    
                    ('RosContext.outputs:context', 'RosSubCMDVel.inputs:context'),
                    ('RosContext.outputs:context', 'RosSubCMDVel.inputs:context'),
                    ('RosContext.outputs:context', 'PubDelayTime.inputs:context'),
                    
                    ('ReadIMU.outputs:angVel', 'PubIMU.inputs:angularVelocity'),
                    ('ReadIMU.outputs:linAcc', 'PubIMU.inputs:linearAcceleration'),
                    ('ReadIMU.outputs:orientation', 'PubIMU.inputs:orientation'),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ('IsaacClock.inputs:resetOnStop', True),
                    ('RosSubCMDVel.inputs:topicName', '/cmd_vel'),
                    
                    ('ReadIMU.inputs:imuPrim', [imu.prim_path]),
                    ('ReadIMU.inputs:readGravity', True),
                    ('PubIMU.inputs:topicName', '/imu_data'),  
                    
                    ('PubDelayTime.inputs:messageName', 'Float32'),
                    ('PubDelayTime.inputs:messagePackage', 'std_msgs'),
                    ('PubDelayTime.inputs:topicName', '/time_threshold'),
                    ('PubDelayTime.inputs:data', _time_threshold),
                ]
            }
        )
    except Exception as e:
        print(' ERROR ----------------------------')
        print(e)
        print(' END ERROR ------------------------')



######################################## Global Variables
_time_threshold = 1.0
_time = 0
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



######################################## Stage Setups
# Add H1
assets_root_path = get_assets_root_path()
h1 = H1FlatTerrainPolicy(
    prim_path= "/World/H1",
    name= "H1",
    usd_path= assets_root_path + "/Isaac/Robots/Unitree/H1/h1.usd",
    position= np.array([0, 0, 1.05]), 
    orientation= euler_angles_to_quat(np.array([0, 0, np.radians(45)])),
    )

# Add IMU
IMU = IMUSensor(
    prim_path= '/World/H1/imu_link/IMU',
    name= 'h1_imu',
    frequency= 500,
    translation= np.array([0, 0, 0]),
    orientation= np.array([1, 0, 0, 0]),
    linear_acceleration_filter_size= 35,
    angular_velocity_filter_size= 25,
    orientation_filter_size= 25,
)


### Initiate Functions
ros2_setup(IMU)


### Reset World
world.reset()



######################################## Infinite Loop
while True:
    world.step(render= True)
    
    if _time > _time_threshold:
        cmd_lin_vel = cartesian_vector(og.Controller.attribute(f"{ros_graph_path}/RosSubCMDVel.outputs:linearVelocity").get())
        cmd_ang_vel = orientation_rpy(og.Controller.attribute(f"{ros_graph_path}/RosSubCMDVel.outputs:angularVelocity").get())
        _base_command = [cmd_lin_vel.x, 0.0, cmd_ang_vel.yaw]
    
    if _physics_ready and not world.is_stopped():
        h1.forward(world.get_physics_dt(), _base_command)
    elif world.is_stopped():
        _physics_ready = False
    else:
        print('reset')
        _physics_ready = True
        h1.initialize()
        h1.post_reset()
        h1.robot.set_joints_default_state(h1.default_pos)
        _lin_vel.set((0, 0, 0))
        _base_command = np.zeros(3)

    

    value = IMU.get_current_frame(read_gravity= True)
    _time = value['time']
    # print(f'time -> {_time}')
    if _time > _time_threshold:
        dt = _time - _prev_t
        
        _oren.set(quat_to_euler_angles(value['orientation']))
        
        _lin_acc.set(value['lin_acc'])
        _lin_acc.rotate(_oren.get(), neglect_gravity= True)
        
        
        _lin_vel.set(dt * _lin_acc.get(), inc= True)
        print('CMD ->', [round(i, 1) for i in _base_command])
        # print('YAW:', _oren.get_deg(dec= 1)[2])
        # _lin_vel.print()
        print('--------------------------------------')
    else:
        print('Waiting for sensor initiation...')
    _prev_t = _time
        
simulation_app.close()



