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
    
    def get_camera_node(package, executable):
        return ComposableNode(
            package=package,
            plugin=executable,
            name='camera_node',
            parameters=[node_params],
            #emulate_tty=True,
            #output='both',
            extra_arguments=[{'use_intra_process_comms': True}],
        )

    #def get_camera_detector_container(camera_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                camera_node,
                ComposableNode(
                    package='armor_detector',
                    plugin='rm_auto_aim::ArmorDetectorNode',
                    name='armor_detector',
                    parameters=[node_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', '--log-level',
                           'armor_detector:='+launch_params['detector_log_level']],
            on_exit=Shutdown(),
        )

    hik_camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')
    mv_camera_node = get_camera_node('mindvision_camera', 'mindvision_camera::MindvisionCameraNode')

    if (launch_params['camera'] == 'hik'):
        cam_detector = hik_camera_node
    elif (launch_params['camera'] == 'mv'):
        cam_detector = mv_camera_node
        
    detector_node = ComposableNode(
        package='left_armor_detector',
        plugin='rm_left_auto_aim::ArmorDetectorNode',
        name='left_armor_detector',
        parameters=[node_params],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    delay_detector_node = TimerAction(
        period=1.5,
        actions=[detector_node],
    )

    trajectory_node = Node(
        package='mechax_trajectory',
        executable='mechax_trajectory',
        name='mechax_trajectory',
        output='both',
        emulate_tty=True,
        on_exit=Shutdown(),
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
    
    delay_tracker_node = TimerAction(
        period=1.5,
        actions=[tracker_node],
    )

    # delay_trajectory_node = TimerAction(
    #     period=2.5,
    #     actions=[trajectory_node],
    # )

    right_detector_node = Node(
        package='right_armor_detector',
        executable='right_armor_detector_node',
        name='right_armor_detector',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        arguments=['--ros-args', '--log-level',
                   'right_armor_detector:='+launch_params['detector_log_level']],
    )

    right_tracker_node = Node(
        package='right_armor_tracker',
        executable='right_armor_tracker_node',
        name='right_armor_tracker',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        arguments=['--ros-args', '--log-level',
                   'right_armor_tracker:='+launch_params['tracker_log_level']],
    )

    delay_right_detector_node = TimerAction(
        period=1.5,
        actions=[right_detector_node],
    )

    delay_right_tracker_node = TimerAction(
        period=1.5,
        actions=[right_tracker_node],
    )

    front_assist_detector_node = Node(
        package='front_assist_armor_detector',
        executable='front_assist_armor_detector_node',
        name='front_assist_armor_detector',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        arguments=['--ros-args', '--log-level',
                   'assist_armor_detector:='+launch_params['front_assist_detector_log_level']],
    )

    front_assist_tracker_node = Node(
        package='front_assist_armor_tracker',
        executable='front_assist_armor_tracker_node',
        name='front_assist_armor_tracker',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        arguments=['--ros-args', '--log-level',
                   'assist_armor_tracker:='+launch_params['front_assist_tracker_log_level']],
    )

    delay_front_assist_detector_node = TimerAction(
        period=1.5,
        actions=[front_assist_detector_node],
    )

    delay_front_assist_tracker_node = TimerAction(
        period=1.5,
        actions=[front_assist_tracker_node],
    )

    back_assist_detector_node = Node(
        package='back_assist_armor_detector',
        executable='back_assist_armor_detector_node',
        name='back_assist_armor_detector',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        arguments=['--ros-args', '--log-level',
                   'back_assist_armor_detector:='+launch_params['back_assist_detector_log_level']],
    )

    back_assist_tracker_node = Node(
        package='back_assist_armor_tracker',
        executable='back_assist_armor_tracker_node',
        name='back_assist_armor_tracker',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        arguments=['--ros-args', '--log-level',
                   'back_assist_armor_tracker:='+launch_params['back_assist_tracker_log_level']],
    )

    delay_back_assist_detector_node = TimerAction(
        period=1.5,
        actions=[back_assist_detector_node],
    )

    delay_back_assist_tracker_node = TimerAction(
        period=1.5,
        actions=[back_assist_tracker_node],
    )

    # delay_serial_node 会以1.5秒的周期触发执行串口驱动节点，
    # 而 delay_tracker_node 会以2.0秒的周期触发执行追踪节点。
    delay_serial_node = TimerAction(
        period=1.5,
        actions=[serial_driver_node],
    )

    outpost_node = Node(
        package='outpost',
        executable='outpost',
        name='outpost',
        output='both',
        emulate_tty=True,
    )

    delay_outpost_node = TimerAction(
        period=1.5,
        actions=[outpost_node],
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
                #openvino_detector_node,
            ],
            output='both',
    )
    print(container)

    return LaunchDescription([
        robot_state_publisher,
        #cam_detector,
        #delay_detector_node,
        container,
        delay_tracker_node,
        trajectory_node,
        #delay_right_detector_node,
        #delay_right_tracker_node,
        #delay_front_assist_detector_node,
        #delay_front_assist_tracker_node,
        #delay_back_assist_detector_node,
        #delay_back_assist_tracker_node,
        delay_serial_node,
        #delay_outpost_node,
    ])
