# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import argparse
import sys

import carb
#carb.settings.get_settings().set("/app/show_developer_preference_section", True)
from omni.isaac.kit import SimulationApp

parser = argparse.ArgumentParser(description="main script .")
parser.add_argument("--usdpath", type=str, default="/home/student/Documents/tang/VisionRobot/isaac_example/hospital_2.usd", help="usd file to be loaded")
parser.add_argument('-hl', '--headless', action='store_true', help="headless ")
args = parser.parse_args()

simulation_app = SimulationApp({"headless": args.headless})

import omni
import omni.kit.viewport.utility
import omni.replicator.core as rep
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils import nucleus, stage
from omni.isaac.core.utils.extensions import enable_extension
from pxr import Gf


# enable ROS2 bridge extension
enable_extension("omni.isaac.ros2_bridge")

simulation_app.update()


# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = nucleus.get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

simulation_app.update()
import carb
carb.settings.get_settings().set("/app/show_developer_preference_section", True)
# Loading the simple_room environment

#stage.add_reference_to_stage("file:/home/student/Documents/tang/VisionRobot/isaac_example/simple_room_lidar.usd", "/myroom")

#stage.add_reference_to_stage(assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd", "/background")
#    assets_root_path + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd", "/background"

omni.usd.get_context().open_stage(args.usdpath) #"/home/student/Documents/tang/VisionRobot/isaac_example/hospital_2.usd")

import omni.isaac.core.utils.stage as stage_utils

#stage_utils.open_stage("/home/student/Documents/tang/VisionRobot/isaac_example/turtlebot_tutorial.usd")

simulation_app.update()
prim0 = get_prim_at_path("/sensor")
#print("bbbbbb",  prim0, prim0.GetProperty("Transform"))


# Create the lidar sensor that generates data into "RtxSensorCpu"
# Sensor needs to be rotated 90 degrees about X so that its Z up

# Possible options are Example_Rotary and Example_Solid_State
# drive sim applies 0.5,-0.5,-0.5,w(-0.5), we have to apply the reverse
_, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path="/sensor",
    parent=None,
    config="Example_Rotary",
    translation=(0, 0, 1.0),
    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
)

prim0 = get_prim_at_path("/sensor")
#print("bbbbbb",  prim0, prim0.GetProperty("Transform"))
# RTX sensors are cameras and must be assigned to their own render product
hydra_texture = rep.create.render_product(sensor.GetPath(), [1, 1], name="Isaac")

simulation_context = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)
simulation_app.update()

# Create Point cloud publisher pipeline in the post process graph
writer = rep.writers.get("RtxLidar" + "ROS2PublishPointCloud")
writer.initialize(topicName="point_cloud", frameId="sim_lidar")
writer.attach([hydra_texture])

# Create the debug draw pipeline in the post process graph
writer = rep.writers.get("RtxLidar" + "DebugDrawPointCloud")
writer.attach([hydra_texture])


# Create LaserScan publisher pipeline in the post process graph
writer = rep.writers.get("RtxLidar" + "ROS2PublishLaserScan")
writer.initialize(topicName="laser_scan", frameId="sim_lidar")
#writer.attach([hydra_texture])

simulation_app.update()

simulation_context.play()

while simulation_app.is_running():
    simulation_app.update()

# cleanup and shutdown
simulation_context.stop()
simulation_app.close()
