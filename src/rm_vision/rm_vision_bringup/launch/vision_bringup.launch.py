import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))


def generate_launch_description():

    from common import node_params, launch_params, robot_state_publisher, tracker_node
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            parameters=[node_params],
            #emulate_tty=True,
            #output='both',
            extra_arguments=[{'use_intra_process_comms': True}],
        )

    hik_camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')
    mv_camera_node = get_camera_node('mindvision_camera', 'mindvision_camera::MVCameraNode')

    if (launch_params['camera'] == 'hik'):
        cam_detector = hik_camera_node
    elif (launch_params['camera'] == 'mv'):
        cam_detector = mv_camera_node

    detector_node = ComposableNode(
        package='armor_detector',
        plugin='rm_auto_aim::ArmorDetectorNode',
        name='armor_detector',
        parameters=[node_params],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        name='serial_driver',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        on_exit=Shutdown(),
        ros_arguments=['--ros-args', '--log-level',
                       'serial_driver:='+launch_params['serial_log_level']],
    )

    trajectory_node = Node(
        package='mechax_trajectory',
        executable='mechax_trajectory',
        name='mechax_trajectory',
        output='both',
        emulate_tty=True,
        on_exit=Shutdown(),
    )

    # delay_serial_node 会以1.5秒的周期触发执行串口驱动节点，
    # 而 delay_tracker_node 会以2.0秒的周期触发执行追踪节点。
    delay_serial_node = TimerAction(
        period=1.5,
        actions=[serial_driver_node],
    )

    delay_tracker_node = TimerAction(
        period=2.0,
        actions=[tracker_node],
    )

    delay_trajectory_node = TimerAction(
        period=2.0,
        actions=[trajectory_node],
    )

    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='image_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                cam_detector,
                detector_node,
            ],
            output='both',
    )
    print(container)

    return LaunchDescription([
        robot_state_publisher, # 可视化
        container,          # 相机+detector
        delay_serial_node,     # 串口通信
        delay_tracker_node,    # tracker
        delay_trajectory_node, # 轨迹规划,弹道解算
    ])
