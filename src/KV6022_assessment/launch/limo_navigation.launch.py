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
    pkg_name = 'KV6022_assessment'
    map_subpath = 'maps/potholes_20mm.yaml'
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


    # Lifecycle Manager - makes handling the map server less tricky
    node_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        parameters=[{'node_names':['example_map_server', 'amcl'], 'autostart': True}] # add other parameters here if required
    )
   


    all_config = os.path.join(get_package_share_directory(pkg_name),'params','nav2_params.yaml')
    
    # https://docs.nav2.org/configuration/packages/configuring-amcl.html
    # AMCL (Adaptive Monte Carlo Localisation)
    node_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[all_config] 
    )

    # Necessary fixes
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[all_config],
        remappings=remappings,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[all_config],
        remappings=remappings,
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=remappings,
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[all_config],
        remappings=remappings,
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[all_config],
    )

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower'
    ]

    # Lifecycle Node Manager to automatically start lifecycles nodes (from list)
    node_lifecycle_manager2 = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}],
    )

    rviz_config_dir = os.path.join(get_package_share_directory(pkg_name),'rviz','limo_navigation.rviz')
    start_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}],
        output='screen')


    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(node_map_server)
    ld.add_action(node_amcl)
    ld.add_action(node_lifecycle_manager)

    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(behavior_server)
    ld.add_action(bt_navigator)
    ld.add_action(waypoint_follower)
    ld.add_action(node_lifecycle_manager2)

    ld.add_action(start_rviz2)
   
    return ld