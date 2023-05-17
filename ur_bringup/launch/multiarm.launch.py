
import launch
import launch_ros.actions

def generate_launch_description():
    # Load the URDF model
    model = launch.substitutions.ThisLaunchFileDir() + 'ur_bringup/launch/ur3e.launch.py'
    robot_description = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', model])}]
    )

    # Launch MoveIt2
    robot1_move_group = launch_ros.actions.Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='robot1_move_group',
        output='screen',
        parameters=[{'move_group': 'robot1_arm'}, {'robot_description': Command(['xacro ', model])},
                    {'planning_plugin': 'ompl_interface/OMPLPlanner'}]
    )

    robot2_move_group = launch_ros.actions.Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='robot2_move_group',
        output='screen',
        parameters=[{'move_group': 'robot2_arm'}, {'robot_description': Command(['xacro ', model])},
                    {'planning_plugin': 'ompl_interface/OMPLPlanner'}]
    )

    # Return the launch description
    return launch.LaunchDescription([
        robot_description,
        robot1_move_group,
        robot2_move_group
    ])
