from launch import LaunchDescription
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config_dir = os.path.join(get_package_share_directory('manipulator_control_pkg'),('config'))

    config = os.path.join(config_dir, 'green_controller_manipulator_control.yaml')
    
    ld = LaunchDescription()

    
    ld.add_action(
        launch_ros.actions.Node(
            package='manipulator',
            executable='manipulator',
        )
    )
    
    ld.add_action(
        launch_ros.actions.Node(
            namespace="manipulator/shoulder", 
            package='hw_interface',
            executable='motor',
            parameters=[{'arbitration_id': 0x146}]  
        )
    )
    
    ld.add_action(
        launch_ros.actions.Node(
            namespace="manipulator/elbow", 
            package='hw_interface',
            executable='motor',
            parameters=[{'arbitration_id': 0x145}]  
        )
    )
        
    ld.add_action(
        launch_ros.actions.Node(
            namespace="manipulator/wrist_roll", 
            package='hw_interface',
            executable='motor',
            parameters=[{'arbitration_id': 0x149}]  
        )
    )
    
    ld.add_action(
        launch_ros.actions.Node(
            namespace="manipulator/wrist_pitch", 
            package='hw_interface',
            executable='motor',
            parameters=[{'arbitration_id': 0x148}]  
        )
    )

    ld.add_action(
        launch_ros.actions.Node(
            namespace="manipulator/linear_rail", 
            package='hw_interface',
            executable='motor',
            parameters=[{'arbitration_id': 0x147}]  
        )
    )
    
    return ld
