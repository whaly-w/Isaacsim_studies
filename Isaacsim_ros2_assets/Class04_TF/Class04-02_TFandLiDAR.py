from isaacsim import SimulationApp
CONFIG = {"renderer": "RayTracedLighting", "headless": False}
simulation_app = SimulationApp(CONFIG)

import omni
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils import stage, extensions, nucleus
import omni.graph.core as og
import omni.replicator.core as rep

from isaacsim.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
from pxr import Gf


# Enable ROS2 bridge extension
extensions.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()
world = World(stage_units_in_meters=1.0)

# Loading the environment
BACKGROUND_STAGE_PATH = "/World/background"
BACKGROUND_USD_PATH = '/Isaac/Environments/Simple_Room/simple_room.usd'
stage.add_reference_to_stage(nucleus.get_assets_root_path() + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)
XFormPrim(prim_path= '/World/background').set_world_pose(position= [0, 0, 0.75])



############### functions for setting up publishers. ###############
def publish_tf():
    try:
        ros_camera_graph_path = "/Graph/TFGraph"
        (ros_camera_graph, _, _, _) = og.Controller.edit(
            {
                "graph_path": ros_camera_graph_path,
                "evaluator_name": "execution",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("IsaacClock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("RosContext", "isaacsim.ros2.bridge.ROS2Context"),
                    ("OdomCal", 'isaacsim.core.nodes.IsaacComputeOdometry'),
                    ('TFOdomRobot', 'isaacsim.ros2.bridge.ROS2PublishRawTransformTree'),
                    ('TFWorldOdom', 'isaacsim.ros2.bridge.ROS2PublishRawTransformTree'),
                    ('TFRobotLinks', 'isaacsim.ros2.bridge.ROS2PublishTransformTree'),
                ],
                og.Controller.Keys.CONNECT: [
                    ('OnPlaybackTick.outputs:tick', 'OdomCal.inputs:execIn'),
                    ('OnPlaybackTick.outputs:tick', 'TFOdomRobot.inputs:execIn'),
                    ('OnPlaybackTick.outputs:tick', 'TFWorldOdom.inputs:execIn'),
                    ('OnPlaybackTick.outputs:tick', 'TFRobotLinks.inputs:execIn'),
                    
                    ('RosContext.outputs:context', 'TFOdomRobot.inputs:context'),
                    ('RosContext.outputs:context', 'TFWorldOdom.inputs:context'),
                    ('RosContext.outputs:context', 'TFRobotLinks.inputs:context'),
                    
                    ('IsaacClock.outputs:simulationTime', 'TFOdomRobot.inputs:timeStamp'),
                    ('IsaacClock.outputs:simulationTime', 'TFWorldOdom.inputs:timeStamp'),
                    ('IsaacClock.outputs:simulationTime', 'TFRobotLinks.inputs:timeStamp'),
                    
                    ('OdomCal.outputs:orientation', 'TFOdomRobot.inputs:rotation'),
                    ('OdomCal.outputs:position', 'TFOdomRobot.inputs:translation'),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ('OdomCal.inputs:chassisPrim', ['/World/Turtlebot3/a___namespace_base_footprint']),
                    
                    ('TFOdomRobot.inputs:parentFrameId', 'odom'),
                    ('TFOdomRobot.inputs:childFrameId', 'a___namespace_base_link'),
                    
                    ('TFWorldOdom.inputs:parentFrameId', 'world'),
                    ('TFWorldOdom.inputs:childFrameId', 'odom'),
                    ('TFWorldOdom.inputs:translation', [-2.8, 0, 0.2]),
                    
                    ('TFRobotLinks.inputs:parentPrim', ['/World/Turtlebot3/a___namespace_base_footprint/a___namespace_base_link']),
                    ('TFRobotLinks.inputs:targetPrims', ['/World/Turtlebot3/a___namespace_wheel_left_link',
                                                         '/World/Turtlebot3/a___namespace_wheel_right_link',
                                                         '/World/Turtlebot3/a___namespace_base_footprint',
                                                         '/World/Turtlebot3/a___namespace_base_footprint/a___namespace_base_link/a___namespace_imu_link',
                                                         '/World/Turtlebot3/a___namespace_base_footprint/a___namespace_base_link/a___namespace_base_scan',
                                                         '/World/Turtlebot3/a___namespace_base_footprint/a___namespace_base_link/a___namespace_caster_back_link'])
                ]
            }
        )
    except Exception as e:
        print(' ERROR ----------------------------')
        print(e)
        print(' END ERROR ------------------------')

# Publish data directly using ActionGraph
def publish_laser_data_RtxLidarHelper(sensor):
    try:
        ros_camera_graph_path = "/Graph/LiDARPublishGraph"
        (ros_camera_graph, _, _, _) = og.Controller.edit(
            {
                "graph_path": ros_camera_graph_path,
                "evaluator_name": "execution",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("SingleRun", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                    ("RosContext", "isaacsim.ros2.bridge.ROS2Context"),
                    ("CreateRenderProduct", 'isaacsim.core.nodes.IsaacCreateRenderProduct'),
                    ('RosRTXLidarHelper_01', 'isaacsim.ros2.bridge.ROS2RtxLidarHelper'),
                    ('RosRTXLidarHelper_02', 'isaacsim.ros2.bridge.ROS2RtxLidarHelper'),
                ],
                og.Controller.Keys.CONNECT: [
                    ('OnPlaybackTick.outputs:tick', 'SingleRun.inputs:execIn'),
                    
                    ('SingleRun.outputs:step', 'CreateRenderProduct.inputs:execIn'),
                    
                    ('CreateRenderProduct.outputs:execOut', 'RosRTXLidarHelper_01.inputs:execIn'),
                    ('CreateRenderProduct.outputs:renderProductPath', 'RosRTXLidarHelper_01.inputs:renderProductPath'),
                    ('CreateRenderProduct.outputs:execOut', 'RosRTXLidarHelper_02.inputs:execIn'),
                    ('CreateRenderProduct.outputs:renderProductPath', 'RosRTXLidarHelper_02.inputs:renderProductPath'),
                    
                    ('RosContext.outputs:context', 'RosRTXLidarHelper_01.inputs:context'),
                    ('RosContext.outputs:context', 'RosRTXLidarHelper_02.inputs:context'),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ('CreateRenderProduct.inputs:cameraPrim', [sensor.GetPath()]),
                    
                    ('RosRTXLidarHelper_01.inputs:frameId', 'a___namespace_base_scan'),
                    ('RosRTXLidarHelper_01.inputs:topicName', 'laser_scan'),
                    
                    ('RosRTXLidarHelper_02.inputs:frameId', 'a___namespace_base_scan'),
                    ('RosRTXLidarHelper_02.inputs:topicName', 'point_cloud'),
                    ('RosRTXLidarHelper_02.inputs:type', 'point_cloud'),
                    ('RosRTXLidarHelper_02.inputs:fullScan', False),
                ]
            }
        )
    except Exception as e:
        print(' ERROR ----------------------------')
        print(e)
        print(' END ERROR ------------------------')
        
# Publish data using extensions
def publish_laser_data(sensor):
    hydra_texture = rep.create.render_product(sensor.GetPath(), [1, 1], name="Isaac")
    
    writer = rep.writers.get("RtxLidar" + "ROS2PublishPointCloud")
    
    writer.initialize(topicName="laser_scan", frameId="a___namespace_base_scan")
    writer.initialize(opicName="point_cloud", frameId="a___namespace_base_scan")
    writer = rep.writers.get("RtxLidar" + "ROS2PublishLaserScan")
    writer.attah([hydra_texture])
    writer.attach([hydra_texture])    
    
    

############### Add objects to the stage ###############
# Add Turtlebot 
add_reference_to_stage(usd_path= '/home/whaly/isaac_ws/Isaacsim_ros2_assets/Class01_ControlTurtleBot/turtlebot3_burger/turtlebot3_burger.usd', prim_path= '/World/Turtlebot3')  
XFormPrim(prim_path= '/World/Turtlebot3').set_world_pose(position= [-2.8, 0, 0.2])

# Create and add RTX LiDAR Prim
_, sensor = omni.kit.commands.execute("IsaacSensorCreateRtxLidar",
    path= "/World/Turtlebot3/a___namespace_base_footprint/a___namespace_base_link/a___namespace_base_scan/LiDAR",
    parent= None,
    config= "Example_Rotary",
    translation= (0, 0, 0),
    orientation= Gf.Quatd(1.0, 0.0, 0.0, 0.0),
)



############### Calling publishing functions ###############
approx_freq = 30
# publish_laser_data(sensor)
publish_laser_data_RtxLidarHelper(sensor)
publish_tf()



####################################################################
# Initialize physics
world.reset()

while simulation_app.is_running():
    world.step(render=True)

world.stop()
simulation_app.close()