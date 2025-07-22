import carb
from isaacsim import SimulationApp
import sys

BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"

CONFIG = {"renderer": "RayTracedLighting", "headless": False}

# Example ROS2 bridge sample demonstrating the manual loading of stages and manual publishing of images
simulation_app = SimulationApp(CONFIG)
import omni
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils import stage, extensions, nucleus
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd

from isaacsim.core.utils.prims import set_targets
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.nodes.scripts.utils import set_target_prims

# Enable ROS2 bridge extension
extensions.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

world = World(stage_units_in_meters=1.0)

# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = nucleus.get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Loading the environment
stage.add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)


###### Camera helper functions for setting up publishers. ########
def publish_camera_tf(camera: Camera): 
    camera_prim = camera.prim_path

    if not is_prim_path_valid(camera_prim):
        raise ValueError(f"Camera path '{camera_prim}' is invalid.")

    try:
        # Generate the camera_frame_id. OmniActionGraph will use the last part of
        # the full camera prim path as the frame name, so we will extract it here
        # and use it for the pointcloud frame_id.
        camera_frame_id= camera_prim.split("/")[-1]

        # Generate an action graph associated with camera TF publishing.
        ros_camera_graph_path = "/CameraTFActionGraph"

        # If a camera graph is not found, create a new one.
        if not is_prim_path_valid(ros_camera_graph_path):
            (ros_camera_graph, _, _, _) = og.Controller.edit(
                {
                    "graph_path": ros_camera_graph_path,
                    "evaluator_name": "execution",
                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
                },
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnTick"),
                        ("IsaacClock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("RosPublisher", "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnTick.outputs:tick", "RosPublisher.inputs:execIn"),
                        ("IsaacClock.outputs:simulationTime", "RosPublisher.inputs:timeStamp"),
                    ]
                }
            )

        # Generate 2 nodes associated with each camera: TF from world to ROS camera convention, and world frame.
        og.Controller.edit(
            ros_camera_graph_path,
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("PublishTF_"+camera_frame_id, "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ("PublishRawTF_"+camera_frame_id+"_world", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PublishTF_"+camera_frame_id+".inputs:topicName", "/tf"),
                    
                    ("PublishRawTF_"+camera_frame_id+"_world.inputs:topicName", "/tf"),
                    ("PublishRawTF_"+camera_frame_id+"_world.inputs:parentFrameId", camera_frame_id),
                    ("PublishRawTF_"+camera_frame_id+"_world.inputs:childFrameId", camera_frame_id+"_world"),
                    # Static transform from ROS camera convention to world (+Z up, +X forward) convention:
                    ("PublishRawTF_"+camera_frame_id+"_world.inputs:rotation", [0.5, -0.5, 0.5, 0.5]),
                ],
                og.Controller.Keys.CONNECT: [
                    (ros_camera_graph_path+"/OnTick.outputs:tick",
                        "PublishTF_"+camera_frame_id+".inputs:execIn"),
                    (ros_camera_graph_path+"/OnTick.outputs:tick",
                        "PublishRawTF_"+camera_frame_id+"_world.inputs:execIn"),
                    (ros_camera_graph_path+"/IsaacClock.outputs:simulationTime",
                        "PublishTF_"+camera_frame_id+".inputs:timeStamp"),
                    (ros_camera_graph_path+"/IsaacClock.outputs:simulationTime",
                        "PublishRawTF_"+camera_frame_id+"_world.inputs:timeStamp"),
                ],
            },
        )
    except Exception as e:
        print(e)

    # Add target prims for the USD pose. All other frames are static.
    set_target_prims(
        primPath= ros_camera_graph_path+"/PublishTF_"+camera_frame_id,
        inputName= "inputs:targetPrims",
        targetPrimPaths= [camera_prim],
    )
    return

def publish_camera_info(camera: Camera, freq): 
    from isaacsim.ros2.bridge import read_camera_info
    
    render_product = camera._render_product_path
    step_size = int(60/freq)
    topic_name = f'/{camera.name}_info'
    queue_size = 1
    node_namespace = ''
    frame_id = camera.prim_path.split('/')[-1]
    
    # Get a ROS2 camera publisher node (ActionGraph)
    writer = rep.writers.get('ROS2PublishCameraInfo')
    camera_info = read_camera_info(render_product_path= render_product)
    writer.initialize(
        # ROS publisher info
        frameId= frame_id,
        nodeNamespace= node_namespace,
        queueSize= queue_size,
        topicName= topic_name,
        # Camera info
        width= camera_info["width"],
        height= camera_info["height"],
        projectionType=camera_info["projectionType"],
        k= camera_info["k"].reshape([1, 9]),
        r= camera_info["r"].reshape([1, 9]),
        p= camera_info["p"].reshape([1, 12]),
        physicalDistortionModel= camera_info["physicalDistortionModel"],
        physicalDistortionCoefficients= camera_info["physicalDistortionCoefficients"],
    )
    writer.attach([render_product])
    
    # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        "PostProcessDispatch" + "IsaacSimulationGate", render_product)
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)
    return

def publish_pointcloud_from_depth(camera: Camera, freq): 
    render_product = camera._render_product_path
    step_size = int(60/freq)
    topic_name = f'/{camera.name}_pointcloud'
    
    # Note, this pointcloud publisher will convert the Depth image to a pointcloud using the Camera intrinsics.
    # This pointcloud generation method does not support semantic labeled objects.
    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
        sd.SensorType.DistanceToImagePlane.name)

    # rv is the input data fed to the ROSPublishPointClound node
    writer = rep.writers.get(rv + "ROS2PublishPointCloud")
    writer.initialize(
        frameId= camera.prim_path.split('/')[-1],
        nodeNamespace= '',
        queueSize= 1,
        topicName= topic_name
        )
    writer.attach([render_product])

    # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        rv + "IsaacSimulationGate", render_product)
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)
    return

def publish_depth(camera: Camera, freq): 
    render_product = camera._render_product_path
    step_size = int(60/freq)
    topic_name = f'{camera.name}_depth'

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                            sd.SensorType.DistanceToImagePlane.name)
    writer = rep.writers.get(rv + "ROS2PublishImage")
    writer.initialize(
        frameId= camera.prim_path.split("/")[-1],
        nodeNamespace= '',
        queueSize= 1,
        topicName= topic_name
    )
    writer.attach([render_product])

    # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        rv + "IsaacSimulationGate", render_product)
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)
    return

def publish_rgb(camera: Camera, freq): 
    render_product = camera._render_product_path
    step_size = int(60/freq)
    topic_name = f'{camera.name}_rgb'

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
    writer = rep.writers.get(rv + "ROS2PublishImage")
    writer.initialize(
        frameId= camera.prim_path.split("/")[-1],
        nodeNamespace= '',
        queueSize= 1,
        topicName= topic_name
    )
    writer.attach([render_product])

    # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        rv + "IsaacSimulationGate", render_product)
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)
    return


###################################################################
# Create a Camera prim. Note that the Camera class takes the position and orientation in the world axes convention.
camera = Camera(
    prim_path="/World/floating_camera",
    position=np.array([-3.11, -1.87, 1.0]),
    frequency=20,
    resolution=(256, 256),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
)
camera.initialize()



############### Calling Camera publishing functions ###############

# Call the publishers.
# Make sure you pasted in the helper functions above, and uncomment out the following lines before running.

approx_freq = 30
publish_camera_tf(camera)
publish_camera_info(camera, approx_freq)
publish_rgb(camera, approx_freq)
publish_depth(camera, approx_freq)
publish_pointcloud_from_depth(camera, approx_freq)

####################################################################

# Initialize physics
world.reset()
world.play()

while simulation_app.is_running():
    world.step(render=True)

world.stop()
simulation_app.close()