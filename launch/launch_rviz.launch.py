import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    package_name = "Penguin"
    
    # Get path to the rviz config file
    rviz_config_file = os.path.join(get_package_share_directory('Penguin'),'config','view.rviz')

    publish_state = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory(package_name),'launch','rsp.launch.py')
      ])
    )

    rviz_launch = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      output='screen'
    )

    publish_joint_state_gui = Node(
      package='joint_state_publisher_gui',
      executable='joint_state_publisher_gui',
      output='screen'
    )

    # Launch!
    return LaunchDescription([
        publish_state,
        rviz_launch,
        publish_joint_state_gui
    ])