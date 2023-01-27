from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='use_library',
            executable='use_library',
            name='use_library',
            output='screen',
            parameters=[
                {'kp' : 5.0},
                {'ki' : 0.0},
                {'kd' : 0.1},
                {'use_sample_time' : False},
                {'sample_time' : 2},
                {'derivative_on_measurement' : False},
                {'remove_ki_bump' : False},
                {'reset_windup' : False},
                {'pid_enabled' : True},
                {'cut_off_freq' : 0},
                {'out_min' : -50},
                {'out_max' : 50},
                {'control_value_topic' : '/control_value_topic'},
                {'actual_state_topic' : '/actual_state_topic'},
                {'set_point_topic' : '/set_point_topic'},
                {'loop_freq' : 10}
            ]
        ),
        Node(
            package='example_system',
            executable='example_system',
            name='example_system',
            output='screen'
        )
    ])