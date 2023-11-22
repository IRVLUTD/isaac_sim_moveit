# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import sys

import carb
import numpy as np
from omni.isaac.kit import SimulationApp

FETCH_STAGE_PATH = "/Fetch"
CONFIG = {"renderer": "RayTracedLighting", "headless": False}

# Example ROS bridge sample demonstrating the manual loading of stages
# and creation of ROS components
simulation_app = SimulationApp(CONFIG)
import omni.graph.core as og
import usdrt.Sdf
from omni.isaac.core import World
from omni.isaac.core.utils import extensions, nucleus, stage, viewports
from pxr import Gf

# enable ROS bridge extension
extensions.enable_extension("omni.isaac.ros_bridge")

simulation_app.update()

# check if rosmaster node is running
# this is to prevent this sample from waiting indefinetly if roscore is not running
# can be removed in regular usage
import rosgraph

if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = nucleus.get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Preparing stage
viewports.set_camera_view(eye=np.array([1.2, 1.2, 0.8]), target=np.array([0, 0, 0.5]))

# Loading the fetch robot USD
# asset_path = "/home/yuxiang/ros_workspace/src/fetch_ros/fetch_description/robots/fetch/fetch.usd"
asset_path = "./fetch/fetch.usd"
stage.add_reference_to_stage(usd_path=asset_path, prim_path=FETCH_STAGE_PATH)

simulation_app.update()

# Creating a action graph with ROS component nodes
try:
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("PublishJointState", "omni.isaac.ros_bridge.ROS1PublishJointState"),
                ("SubscribeJointState", "omni.isaac.ros_bridge.ROS1SubscribeJointState"),
                ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                ("PublishTF", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
                ("PublishClock", "omni.isaac.ros_bridge.ROS1PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnImpulseEvent.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "SubscribeJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "PublishTF.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "ArticulationController.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # Setting the /Fetch target prim to Articulation Controller node
                ("ArticulationController.inputs:usePath", True),
                ("ArticulationController.inputs:robotPath", FETCH_STAGE_PATH),
                ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(FETCH_STAGE_PATH)]),
                ("PublishTF.inputs:targetPrims", [usdrt.Sdf.Path(FETCH_STAGE_PATH)]),
            ],
        },
    )
except Exception as e:
    print(e)


simulation_app.update()

# need to initialize physics getting any articulation..etc
world.initialize_physics()

world.play()

while simulation_app.is_running():

    # Run with a fixed step size
    world.step(render=True)

    # Tick the Publish/Subscribe JointState, Publish TF and Publish Clock nodes each frame
    og.Controller.set(og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True)

world.stop()
simulation_app.close()