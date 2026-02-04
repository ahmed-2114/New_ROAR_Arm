from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('ROAR_pkg')
    urdf_file = os.path.join(pkg_path, 'urdf', 'New_URDF.urdf')

    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

#    joint_state_publisher_node = Node(
#        package='joint_state_publisher_gui',
#        executable='joint_state_publisher_gui',
#        output='screen'
#    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_path, 'urdf', 'config.rviz')],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
#        joint_state_publisher_node,
        rviz_node
    ])