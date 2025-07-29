from isaacsim import SimulationApp
CONFIG = {"renderer": "RayTracedLighting", "headless": False}
simulation_app = SimulationApp(CONFIG)

import numpy as np
import omni
from isaacsim.core.api import World

import omni.graph.core as og
from isaacsim.core.utils import extensions

# Enable ROS2 bridge extension
extensions.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# Create a world + setups
world = World()
world.scene.add_default_ground_plane()


# Create Action Graph
ros_graph_path = "/Graph/ROS2Graph"
(ros_camera_graph, _, _, _) = og.Controller.edit(
    {
        "graph_path": ros_graph_path,
        "evaluator_name": "execution",
        "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
    },
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", 'omni.graph.action.OnPlaybackTick'),
            ('RosContext', 'isaacsim.ros2.bridge.ROS2Context'),
            ('RosSubCMDVel', 'isaacsim.ros2.bridge.ROS2SubscribeTwist'),
        ],
        og.Controller.Keys.CONNECT: [
            ('OnPlaybackTick.outputs:tick', 'RosSubCMDVel.inputs:execIn'),
            ('RosContext.outputs:context', 'RosSubCMDVel.inputs:context'),
        ],
        og.Controller.Keys.SET_VALUES: [
            ('RosSubCMDVel.inputs:topicName', '/cmd_vel'),
        ]
    }
)


# Infinite Loop
world.reset()
while simulation_app.is_running():
    # Retreive data from output of RosSubCMDVel node
    existing_text = og.Controller.attribute(f"{ros_graph_path}/RosSubCMDVel.outputs:linearVelocity").get()
    print("X axis velocity: ", existing_text[0])
    
    world.step(render=True)

world.stop()
simulation_app.close()