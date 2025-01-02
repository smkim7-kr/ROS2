from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the image_proc node with remapping
        Node(
            package='camera',  # The ROS2 package containing the node
            executable='image_proc',  # The name of the executable to run
            name='image_proc_node',  # A unique name for this node instance
            remappings=[
                # Remap the default topic names to match our camera namespace
                ('/image_raw', '/camera/image_raw'),  # Input raw image from camera
                ('/image_rect', '/camera/image_rect'),  # Output rectified image
                ('/image_color', '/camera/image_color'),  # Output color image
                ('/image_mono', '/camera/image_mono')  # Output monochrome image
            ],  
            output='screen'  # Display node output in the console
        )
    ])

