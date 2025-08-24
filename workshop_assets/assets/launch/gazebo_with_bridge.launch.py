from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Caminho relativo ao pacote
    pkg_share = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    
    # Caminho do arquivo SDF
    sdf_path = os.path.join(pkg_share, 'world', 'explore_world.sdf')

    # Caminho do executável lidar_node
    lidar_node_path = os.path.join(pkg_share, 'scripts', 'build', 'lidar_node')
    lidar_node_cwd = os.path.join(pkg_share, 'scripts', 'build')

    # Caminho do arquivo de parâmetros Nav2
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    # Processo para iniciar o Gazebo
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', sdf_path, '--verbose'],
        output='screen'
    )

    # Processo para rodar lidar_node
    lidar_node = ExecuteProcess(
        cmd=[lidar_node_path],
        cwd=lidar_node_cwd,
        output='screen'
    )

    # Bridge cmd_vel
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
        output='screen'
    )

    # Bridge imu
    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU'],
        output='screen'
    )

    # Bridge lidar 2-D view
    bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
        output='screen'
    )
    
    # Bridge lidar_distance - 1D
    bridge_lidar_avg = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan/linear@std_msgs/msg/Float64@ignition.msgs.Double'],
        output='screen'
    )

    # Bridge camera
    bridge_camera_image = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera@sensor_msgs/msg/Image@ignition.msgs.Image'],
        output='screen'
    )

    # Bridge odom
    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry'],
        output='screen'
    )
    
    bridge_vehicle_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/vehicle_blue/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V'],
        remappings=[('/model/vehicle_blue/tf', '/tf')],
        output='screen'
    )

    bridge_vehicle_tf_static = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/vehicle_blue/tf_static@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V'],
        remappings=[('/model/vehicle_blue/tf_static', '/tf_static')],
        output='screen'
    )
    
    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'vehicle_blue/odom'],
        output='screen'
    )

    # Static TF: vehicle_blue/base_link -> base_link
    static_tf_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'vehicle_blue/base_link', 'base_link'],
        output='screen'
    )

    static_tf_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
     # Nav2 bringup node com parâmetro
    nav2_bringup = Node(
        package='nav2_bringup',
        executable='nav2_bringup',
        name='nav2_bringup',
        output='screen',
        parameters=[nav2_params_file]
    )
    
    return LaunchDescription([
        gazebo,
        lidar_node,
        bridge_cmd_vel,
        bridge_imu,
        bridge_lidar,
        bridge_lidar_avg,
        bridge_camera_image,
        bridge_odom,
        bridge_vehicle_tf,
        bridge_vehicle_tf_static,
        static_tf_odom,
    	static_tf_base_link,
    	static_tf_map,
    ])

