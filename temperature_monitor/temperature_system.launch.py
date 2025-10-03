from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # TODO: Declare launch arguments for configurable parameters
    min_temp_arg = DeclareLaunchArgument(
        'min_temp',
        default_value='15.0',
        description='Minimum temperature value'
    )

    max_temp_arg = DeclareLaunchArgument(
        'max_temp',
        default_value='35.0',
        description='Maximum temperature value'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Publishing rate in seconds'
    )

    location_arg = DeclareLaunchArgument(
        'location',
        default_value='DEEC',
        description='Sensor location'
    )

    warning_threshold_arg = DeclareLaunchArgument(
        'warning_threshold',
        default_value='25.0',
        description='Warning temperature threshold'
    )

    critical_threshold_arg = DeclareLaunchArgument(
        'critical_threshold',
        default_value='27.5',
        description='Critical temperature threshold'
    )

    log_file_path_arg = DeclareLaunchArgument(
        'log_file_path',
        default_value='temperature_monitor/temperature_monitor/temperature_log.txt',
        description='Path to the log file'
    )   

    log_frequency_arg = DeclareLaunchArgument(
        'log_frequency',
        default_value='1',
        description='Log every N readings'
    )

    # TODO: Create node actions with parameter mappings
    enhanced_sensor_node = Node(
        package='temperature_monitor',
        executable='publisher_node',
        name='enhanced_temperature_sensor',
        parameters=[{
            'min_temp': LaunchConfiguration('min_temp'),
            'max_temp': LaunchConfiguration('max_temp'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'location': LaunchConfiguration('location')
        }]
    )

    advanced_monitor_node = Node(
        package='temperature_monitor',
        executable='monitor_node',
        name='advanced_monitor_node',
        parameters=[{
            'warning_threshold': LaunchConfiguration('warning_threshold'),
            'critical_threshold': LaunchConfiguration('critical_threshold')
        }]
    )

    temperature_logger_node = Node(
        package='temperature_monitor',
        executable='monitor_node',
        name='temperature_logger',
        parameters=[{
            'log_file_path': LaunchConfiguration('log_file_path'),
            'log_frequency': LaunchConfiguration('log_frequency')
        }]
    )

    # TODO: Return LaunchDescription with all nodes and arguments
    
    return LaunchDescription([
        # Your launch arguments and nodes will go here
        min_temp_arg,
        max_temp_arg,
        publish_rate_arg,
        location_arg,
        warning_threshold_arg,
        critical_threshold_arg,
        log_file_path_arg,
        log_frequency_arg,
        
        enhanced_sensor_node,
        advanced_monitor_node,
        temperature_logger_node
    ])