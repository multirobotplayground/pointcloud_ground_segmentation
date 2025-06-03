import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  ns = LaunchConfiguration('namespace', default='robot_1')
  config_file = os.path.join(get_package_share_directory('point_cloud_segmentation'), 'config', 'ground_segmentation.yaml')

  segmentation_node = Node(
    package='point_cloud_segmentation',
    executable='point_cloud_segmentation',
    output='screen',
    namespace=ns,
    parameters=[config_file,
                {'use_simtime': True}],
    remappings=[],
  )

  ld = LaunchDescription()
  ld.add_action(segmentation_node)

  return ld
