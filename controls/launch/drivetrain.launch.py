from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    
    ld = LaunchDescription()

    ld.add_action(
        launch_ros.actions.Node(
            package='controls',
            executable='drivetrain_debugger',
        )
    )
    
    ld.add_action(
        launch_ros.actions.Node(
            package='controls',
            executable='drivetrain',
        )
    )
    
    ld.add_action(
        launch_ros.actions.Node(
            namespace="drivetrain/front_left", 
            package='controls',
            executable='motor',
            parameters=[{'arbitration_id': 0x144}]  
        )
    )
    
    ld.add_action(
        launch_ros.actions.Node(
            namespace="drivetrain/front_right", 
            package='controls',
            executable='motor',
            parameters=[{'arbitration_id': 0x142}]  
        )
    )
        
    ld.add_action(
        launch_ros.actions.Node(
            namespace="drivetrain/back_left", 
            package='controls',
            executable='motor',
            parameters=[{'arbitration_id': 0x143}]  
        )
    )
    
    ld.add_action(
        launch_ros.actions.Node(
            namespace="drivetrain/back_right", 
            package='controls',
            executable='motor',
            parameters=[{'arbitration_id': 0x141}]  
        )
    )
    
    return ld
