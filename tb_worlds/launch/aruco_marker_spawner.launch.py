import os
import tempfile
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
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
            "model_dir": "aruco_id_80",
            "x": "-1.0",
            "y": "-2.0",
            "z": "0.01",
            "Y": "0",
        },
        {
            "name": "aruco_id_60",
            "model_dir": "aruco_id_60",
            "x": "-1.5",
            "y": "-2.05",
            "z": "0.23",
            "Y": "0",
        },
    ]

    spawn_actions = []
    for m in markers:
        model_dir = os.path.join(pkg_dir, "models", m["model_dir"])
        sdf_path = os.path.join(model_dir, "model.sdf")

        # Read the SDF and replace model:// URIs with absolute paths
        # so the gz_spawn_model create node can resolve meshes
        with open(sdf_path, "r") as f:
            sdf_content = f.read()
        sdf_content = sdf_content.replace(f"model://{m['model_dir']}/", f"{model_dir}/")

        # Write to a temp file (the create node needs a file path)
        tmp_sdf = tempfile.NamedTemporaryFile(
            prefix=f"aruco_{m['name']}_",
            suffix=".sdf",
            mode="w",
            delete=False,
        )
        tmp_sdf.write(sdf_content)
        tmp_sdf.close()

        spawn_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_spawn_launch),
                launch_arguments={
                    "world": "",
                    "file": tmp_sdf.name,
                    "entity_name": m["name"],
                    "x": m["x"],
                    "y": m["y"],
                    "z": m["z"],
                    "Y": m["Y"],
                }.items(),
            )
        )

    # Delay spawning to allow gz-sim to fully start
    delayed_spawn = TimerAction(period=10.0, actions=spawn_actions)

    return LaunchDescription([delayed_spawn])
