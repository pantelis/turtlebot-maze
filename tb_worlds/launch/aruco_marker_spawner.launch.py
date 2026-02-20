import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = get_package_share_directory("tb_worlds")
    gz_spawn_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"),
        "launch",
        "gz_spawn_model.launch.py",
    )

    markers = [
        {
            "name": "aruco_id_80",
            "sdf": os.path.join(pkg_dir, "models", "aruco_id_80", "model.sdf"),
            "x": "-1.0",
            "y": "-2.0",
            "z": "0.01",
            "Y": "0",
        },
        {
            "name": "aruco_id_60",
            "sdf": os.path.join(pkg_dir, "models", "aruco_id_60", "model.sdf"),
            "x": "-1.5",
            "y": "-2.05",
            "z": "0.23",
            "Y": "0",
        },
    ]

    actions = []
    for m in markers:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_spawn_launch),
                launch_arguments={
                    "world": "",
                    "file": m["sdf"],
                    "name": m["name"],
                    "x": m["x"],
                    "y": m["y"],
                    "z": m["z"],
                    "Y": m["Y"],
                }.items(),
            )
        )

    return LaunchDescription(actions)
