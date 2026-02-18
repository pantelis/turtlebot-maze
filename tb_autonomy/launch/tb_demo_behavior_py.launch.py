from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_tb_worlds = get_package_share_directory("tb_worlds")
    default_world_dir = join(pkg_tb_worlds, "maps", "sim_house_locations.yaml")

    return LaunchDescription(
        [
            # Arguments
            DeclareLaunchArgument(
                "location_file",
                default_value=TextSubstitution(text=default_world_dir),
                description="YAML file name containing map locations in the world.",
            ),
            DeclareLaunchArgument(
                "target_color",
                default_value=TextSubstitution(text="blue"),
                description="Target object color (red, green, or blue)",
            ),
            DeclareLaunchArgument(
                "tree_type",
                default_value=TextSubstitution(text="queue"),
                description="Behavior tree type (naive or queue)",
            ),
            DeclareLaunchArgument(
                "enable_vision",
                default_value=TextSubstitution(text="True"),
                description="Enable vision behaviors. If false, do navigation only.",
            ),
            DeclareLaunchArgument(
                "detector_type",
                default_value=TextSubstitution(text="hsv"),
                description="Detector type: hsv (color threshold) or yolo (YOLOv8 via Zenoh)",
            ),
            DeclareLaunchArgument(
                "target_object",
                default_value=TextSubstitution(text="cup"),
                description="COCO class name to search for (yolo mode only)",
            ),
            # Main autonomy node
            Node(
                package="tb_autonomy",
                executable="autonomy_node.py",
                name="autonomy_node_python",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "location_file": LaunchConfiguration("location_file"),
                        "target_color": LaunchConfiguration("target_color"),
                        "tree_type": LaunchConfiguration("tree_type"),
                        "enable_vision": LaunchConfiguration("enable_vision"),
                        "detector_type": LaunchConfiguration("detector_type"),
                        "target_object": LaunchConfiguration("target_object"),
                    }
                ],
            ),
            # Behavior tree visualization
            ExecuteProcess(cmd=["py-trees-tree-viewer", "--no-sandbox"]),
        ]
    )
