import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name = 'hack' #<--- CHANGE ME my_map1.yaml

    static_map_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'challenge1_map.yaml')
    nav2_params_path = os.path.join(get_package_share_directory('hack'), 'config', 'nav2_pure_pursuite.yaml')
    default_rviz_config_path = os.path.join(get_package_share_directory(package_name), 'config', 'hack.rviz')

    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_dir, 'launch')

    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    waypoints= Node(
        package='hack',
        executable='waypont.py',
        name='waypont',
    
    )

    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[LaunchConfiguration('use_sim_time')],
        arguments=['-d', LaunchConfiguration('rvizconfig')])

    

    # Launch them all!
    return LaunchDescription([

        # launch.actions.ExecuteProcess(
        #     cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
        #     output='screen'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                             description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                             description='Absolute path to rviz config file'),

        launch.actions.DeclareLaunchArgument(name='map', default_value=static_map_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument(name='params_file', default_value=nav2_params_path,
                                             description='Full path to the ROS2 parameters file to use for all launched nodes'),
        # launch.actions.DeclareLaunchArgument(name='autostart', default_value='true',
        #                                      description='Automatically startup the nav2 stack'),
        # launch.actions.DeclareLaunchArgument(name='default_bt_xml_filename', default_value=behavior_tree_xml_path,
        #                                      description='Full path to the behavior tree xml file to use'),
        # launch.actions.DeclareLaunchArgument(name='slam', default_value='False',
        #                                      description='Whether to run SLAM'),

        # Launch the ROS 2 Navigation Stack
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
            launch_arguments={'map': LaunchConfiguration('map'),
                              #'slam': LaunchConfiguration('slam'),
                              'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'params_file': LaunchConfiguration('params_file'),
                              #'default_bt_xml_filename': LaunchConfiguration('default_bt_xml_filename'),
                              #'autostart': LaunchConfiguration('autostart')
                              }.items()),

        #rsp,
        rviz_node,
        waypoints,
        #static_transform,
        #spawn_entity,
        #diff_drive_spawner,
        #joint_broad_spawner,
        
    ])
