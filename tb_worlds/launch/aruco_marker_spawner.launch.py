import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    lds = []
    pkg_dir = get_package_share_directory("tb_worlds")
    
    # Define the spawn position (x, y, yaw) for aruco marker
    spawn_locations = {
        "aruco_id_80": (-1.0, -2.0, 0.0, 0.01),
        # "aruco_id_60": (-1.5, -2.05, 0.0, 0.23),   # You can change coordinates here
    }

    for model_name, (x, y, theta,z) in spawn_locations.items():
        model_sdf = os.path.join(pkg_dir, "models", model_name, "model.sdf")
        print(model_sdf)
        lds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("ros_gz_sim"),
                        "launch",
                        "gz_spawn_model.launch.py",
                    )
                ),
                launch_arguments={
                    "world": "",  # use default or modify if needed
                    "file": model_sdf,
                    "name": model_name,
                    "x": str(x),
                    "y": str(y),
                    "z": str(z),  # tiny offset above ground to avoid z-fighting
                    "Y": str(theta),
                }.items(),
            )
        )

    return LaunchDescription(lds)
