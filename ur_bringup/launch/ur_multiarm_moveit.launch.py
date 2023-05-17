import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import AnyLaunchDescriptionSource

from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    ur_arm_1 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ur_bringup"),
                "launch",
                "ur_arm1_moveit.launch.py",
            )
        )
    )
    
    ur_arm_2 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ur_bringup"),
                "launch",
                "ur_arm2_moveit.launch.py",
            )
        )
    )
    return LaunchDescription([ur_arm_1,ur_arm_2])
    
