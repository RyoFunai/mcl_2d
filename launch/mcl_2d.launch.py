from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
  script_directory = Path(__file__).resolve().parent
  
  map_param_path = script_directory / "../config/maps/iscas_museum_map.yaml"
  map_param_path = map_param_path.resolve()
  
  mcl_2d_param_path = script_directory / "../config/param/mcl_2d.yaml"
  mcl_2d_param_path = mcl_2d_param_path.resolve()
  
  rviz_config_file = script_directory / "../config/rviz2/mcl_2d.rviz"
  rviz_config_file = rviz_config_file.resolve()
  

  mcl_2d = Node(
    package='mcl_2d',
    executable='mcl_2d',
    name='mcl_2d',
    output='log',
    parameters=[mcl_2d_param_path, {'map_param_path': str(map_param_path)}]
  )

  map_server = Node(
    package='map_server',
    executable='map_server',
    name='map_server',
    output='screen',
    parameters=[{'yaml_filename': str(map_param_path)}]
  )

  rviz2 = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='log',
    arguments=['-d', str(rviz_config_file)]
  )

  ld = LaunchDescription()
  ld.add_action(mcl_2d)
  # ld.add_action(map_server)
  # ld.add_action(rviz2)

  return ld