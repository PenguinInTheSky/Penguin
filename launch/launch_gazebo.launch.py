import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    package_name = "Penguin"

    rsp = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory(package_name),'launch','rsp.launch.py'
      )]),
      launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo_params_path = os.path.join(
                    get_package_share_directory(package_name),'config','gazebo_params.yaml')

    gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py')]),
        launch_arguments={'extra_gazebo_args': '--ros-args --param-file ' + gazebo_params_path}.items()
    )


    spawn_entity = Node(
      package='gazebo_ros',
      executable='spawn_entity.py',
      arguments=['-topic','robot_description','-entity','whales'],
      output='screen'
    )

    tri_cont_spawner = Node(
      package='controller_manager',
      executable='spawner',
      arguments=["tri_cont"]
    )

    return LaunchDescription([
      rsp,
      gazebo,
      spawn_entity,
      tri_cont_spawner
    ])
