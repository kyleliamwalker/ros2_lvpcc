import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package="imu_ros2_driver",
            executable="imu_data",
            parameters=[{'number_of_imus':4}],
            name='imu_data',
            output="screen",
        ),
        
        Node(
            package="imu_ros2_driver",
            executable="imu_data_converter",
            parameters=[{'imu_id':0}],
            name='imu_data_converter_0',
            output="screen",
        ),
        
        Node(
            package="imu_ros2_driver",
            executable="imu_data_converter",
            parameters=[{'imu_id':1}],
            name='imu_data_converter_1',
            output="screen",
        ),
        Node(
            package="imu_ros2_driver",
            executable="imu_dynamic_model.py",
            name='imu_dynamic_model',
            output="screen",
        ),
        Node(
            package='ros2_lvpcc',
            executable='read_quad_encoder_node.py',
            name='encoder',
            output="screen",
        ),
        Node(
            package='ros2_lvpcc',
            executable='lvpcc_model_node.py',
            name='lvpcc',
            output="screen",
        ),
        # Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     name='rviz2',
        #     arguments=['-d', '/home/kwalker96/ros2_ws/src/imu_controller/rviz_config_array.rviz']
        # ),
    ])

