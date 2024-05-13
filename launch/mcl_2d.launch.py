import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  package_dir = get_package_share_directory('mcl_2d')

  map_file_path = os.path.join(package_dir, 'config', 'maps', 'iscas_museum_map.yaml')
  rviz_config_file = os.path.join(package_dir, 'config', 'rviz2', 'mcl_2d.rviz')

  mcl_2d = Node(package='mcl_2d',
    executable='mcl_2d',
    name='mcl_2d',
    output='log',
  )
  

  map_server = Node(
      package='map_server',
      executable='map_server',
      name='map_server',
      output='screen',
      parameters=[{'yaml_filename': map_file_path}]
  )

  rviz2 = Node(package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='log',
    arguments=['-d', rviz_config_file],
  )
  
  
  ld = LaunchDescription()
  ld.add_action(mcl_2d)
  ld.add_action(map_server)
  ld.add_action(rviz2)

  return ld