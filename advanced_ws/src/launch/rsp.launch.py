import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('advanced_robot'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    # to keep odom -> base_link transform
    # https://answers.ros.org/question/389383/slam_toolbox-message-filter-dropping-message-for-reason-discarding-message-because-the-queue-is-full/
    queuefull_handler = Node(
        package='tf2_ros',
        namespace = 'scan_to_map',
        executable='static_transform_publisher',
        arguments= ["0", "0", "0", "0", "0", "0", "map", "scan"]
    )
    
    # Create fake gui publisher to visualize movable links in rviz
    gui_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_fake_gui'))
    )
    
    # Find the package share directory for the RViz configuration file
    # rviz_config_file = os.path.join(
    #     FindPackageShare("advanced_robot").find("advanced_robot"),
    #     "config",  # Folder in your package containing your RViz config
    #     "tf2_urdf.rviz"  # Name of your RViz configuration file
    # )
    
    # Open Rviz Visualizer (TF, RobotModel)
    rviz_visualizer = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
            # arguments=['-d', rviz_config_file]  # Optionally load a specific RViz config file
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false', # Gazebo needs true in here
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'launch_fake_gui',
            default_value='false', # fake gui publisher
            description='Launch fake gui publisher if true'),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false', # rviz publisher
            description='Launch rviz if true'),
        node_robot_state_publisher,
        # queuefull_handler,
        gui_publisher, 
        rviz_visualizer 
    ])
