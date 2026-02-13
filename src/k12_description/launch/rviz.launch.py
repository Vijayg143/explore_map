from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
#import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('k12_description')

    # Path to xacro
#    xacro_file = os.path.join(pkg_path, 'urdf', 'k12.xacro')

    # Convert xacro → urdf
#    robot_description_config = xacro.process_file(xacro_file)
#    robot_desc = robot_description_config.toxml()

    # Optional RViz config
    rviz_config_file = PathJoinSubstitution([pkg_share, 'config', 'rviz_param.rviz'])


    # =========================
    # Declare arguments
    # =========================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # =========================
    # Nodes
    # =========================


    # RViz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        #arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz2_node
    ])
