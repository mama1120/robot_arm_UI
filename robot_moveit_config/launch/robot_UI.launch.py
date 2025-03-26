from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    # Load MoveIt configuration
    moveit_config = MoveItConfigsBuilder("robot", package_name="robot_moveit_config").to_moveit_configs()
    
    # Define the MoveIt demo launch
    moveit_demo_launch = generate_demo_launch(moveit_config)

    # Define the nodes to launch from robot_control package
    move_home_node = Node(
        package='robot_control',
        executable='move_home',  # Assuming executable is named 'move_home'
        name='move_home_node',
        output='screen'
    )

    move_X_Z_node = Node(
        package='robot_control',
        executable='move_X_Z',  # Assuming executable is named 'move_X_Z'
        name='move_X_Z_node',
        output='screen'
    )
    
    joint_listener_node = Node(
        package='robot_control',
        executable='joint_listener',
        name='joint_listener_node',
        output='screen'
    )
    
    # Return the launch description with all the nodes and demo launch
    return LaunchDescription([
        moveit_demo_launch,
        move_home_node,
        move_X_Z_node,
        joint_listener_node
    ])
