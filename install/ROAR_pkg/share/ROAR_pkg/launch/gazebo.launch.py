from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_desc = get_package_share_directory('my_robot_arm_pkg')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')

    # 1. Setup Paths
    urdf_path = os.path.join(pkg_desc, 'urdf', 'urdf_moveit.urdf') 
    
    # Define absolute path to the YAML file
    # We use os.path.join so we get a string we can use in .replace()
    controller_yaml_absolute = os.path.join(pkg_desc, "config", "joint_controllers.yaml")
    
    mesh_absolute_path = os.path.join(pkg_desc, 'meshes')

    # 2. Process URDF
    # We read the file and replace the 'package://' paths with absolute paths
    # so Gazebo/ros2_control can find them without error.
    with open(urdf_path, 'r') as urdf_file:
        robot_desc_raw = urdf_file.read()

    robot_description = robot_desc_raw.replace(
        "package://my_robot_arm_pkg/meshes",
        mesh_absolute_path
    ).replace(
        "package://my_robot_arm_pkg/config/joint_controllers.yaml",
        controller_yaml_absolute
    )

    # 3. Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 4. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    
    # 5. Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'robot_arm_urdf', 
                   '-topic', '/robot_description'],
        output='screen',
    )

    # 6. Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/empty/model/robot_arm_urdf/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/empty/model/robot_arm_urdf/joint_state', 'joint_states'),
        ],
        output='screen'
    )

    # 7. Spawners
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    load_arm_group_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_group_controller"],
        output="screen"
    )

    load_hand_group_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_group_controller"],
        output="screen"
    )
    

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        bridge,
        load_joint_state_broadcaster,
        load_arm_group_controller,
        load_hand_group_controller,
    ])