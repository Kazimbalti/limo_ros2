import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    ld = LaunchDescription()

     # Specify the name of the package and path to map yaml file
    pkg_name = 'limo_localisation'
    map_subpath = 'maps/map.yaml'
    map_yaml_filepath = os.path.join(get_package_share_directory(pkg_name), map_subpath)


    # map server node
    # Publishes a 2D occupancy grid based on a .pgm (and accompanying .yaml) file
    node_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='example_map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_filepath}] # add other parameters here if required
    )

    amcl_config = os.path.join(get_package_share_directory(pkg_name),'params','amcl.yaml')

    # https://docs.nav2.org/configuration/packages/configuring-amcl.html
    # AMCL (Adaptive Monte Carlo Localisation)
    node_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[amcl_config] 
    )


    # Lifecycle Manager - makes handling the map server less tricky
    node_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        parameters=[{'node_names':['example_map_server', 'amcl'], 'autostart': True}] # add other parameters here if required
    )
   
    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(node_map_server)
    ld.add_action(node_amcl)
    ld.add_action(node_lifecycle_manager)

    return ld
