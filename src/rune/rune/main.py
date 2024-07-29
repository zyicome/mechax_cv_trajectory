from math import sqrt
from sys import stderr

import cv_bridge
import rclpy
import rclpy.duration
import sensor_msgs.msg
import tf2_ros
from cv2 import findContours
from geometry_msgs.msg import Point
from tf2_geometry_msgs import PointStamped
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data

from rune.image_progressing import *
from rune.linear_calculation import *
from rune.rlogging import logger
from rune.power_rune import Blade
from rune.power_rune import filter_by_area
from rune.power_rune import filter_by_hu
from rune.power_rune import predict
from rune.prediction import Predictor

__bridge = cv_bridge.CvBridge()
predictor = Predictor()
__camera_info: sensor_msgs.msg.CameraInfo | None = None
__tf_buffer: tf2_ros.Buffer = None
__tf_listener: tf2_ros.TransformListener = None


def set_camera_info(msg: sensor_msgs.msg.CameraInfo):
    global __camera_info
    __camera_info = msg


def callback(msg: sensor_msgs.msg.Image, publisher: Publisher):
    """
    处理图像消息，进行图像处理和预测，并发布3D点位信息。

    参数:
    - msg: sensor_msgs.msg.Image类型，表示接收到的图像消息。
    - publisher: Publisher类型，用于发布处理后的3D点位信息。
    """
    try:
        image = __bridge.imgmsg_to_cv2(msg)  # 将图像消息转换为OpenCV图像格式
    except cv_bridge.CvBridgeError as e:
        print(e, file=stderr)
        return
    gray = to_gray(image)  # 将图像转换为灰度图像
    binary = to_binary(gray)  # 对灰度图像进行二值化处理

    # 查找图像中的轮廓
    contours, *_ = findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 计算轮廓信息
    infos = [
        ContourInfo(contour)
        for contour in contours
    ]
    # 根据面积和Hu矩过滤轮廓信息
    infos = filter_by_area(infos)
    infos = filter_by_hu(infos)
    if len(infos) == 0:
        logger.debug('未检测到能量机关')  # 没有检测到目标，记录调试信息
        return
    if len(infos) > 1:
        logger.warning(f'检测到 {len(infos)} 个能量机关，仅使用最形状接近的一个')  # 发现多个目标，记录警告信息
    # 选择第一个轮廓信息进行处理
    blade = Blade(infos[0])
    predictor.update(blade.vector)  # 更新预测器
    radian = predictor.predict()  # 进行预测，获取角度信息
    predicted_vector = predict(blade.vector, radian)  # 根据预测角度计算预测向量
    if __camera_info is None:
        logger.error('没有相机信息，无法计算3D点位信息')  # 没有相机信息，记录警告信息
        return

    # 计算3D点位信息
    point3d = {
        'x': 1,
        'y': -(predicted_vector[0] + blade.center[0] - __camera_info.k[2]) / __camera_info.k[0],
        'z': -(predicted_vector[1] + blade.center[1] - __camera_info.k[5]) / __camera_info.k[4],
    }
    # 计算距离并调整3D点位
    distance = (__camera_info.k[0] + __camera_info.k[4]) / 2 / sqrt(
        predicted_vector[0] ** 2
        + predicted_vector[1] ** 2
    ) * 0.7 * cfg.distance_correction_ratio

    point3d['x'] *= distance
    point3d['y'] *= distance
    point3d['z'] *= distance

    # 创建并配置点消息
    point_msg = PointStamped(
        header=msg.header,
        point=Point(**point3d)
    )
    point_msg.header.frame_id = "camera_link"

    # 尝试将点消息从相机坐标系转换到odom坐标系并发布
    try:
        # 时间戳设置为0，获取最新的变换
        # point_msg.header.stamp.sec = 0
        # point_msg.header.stamp.nanosec = 0
        point_msg = __tf_buffer.transform(point_msg, "odom",rclpy.duration.Duration(seconds=0))
        publisher.publish(point_msg)
    except Exception as e:
        logger.warning(f"无法将坐标从 camera_link 转换到 odom：{e}")


def main():
    rclpy.init()
    node = rclpy.create_node('rune')
    global __tf_buffer
    global __tf_listener
    __tf_buffer = tf2_ros.Buffer()
    __tf_listener = tf2_ros.TransformListener(__tf_buffer, node)

    publisher: Publisher = node.create_publisher(
        PointStamped,
        '/prediction',
        10
    )
    node.create_subscription(
        sensor_msgs.msg.Image,
        '/image_raw',
        lambda msg: callback(msg, publisher),
        qos_profile_sensor_data
    )
    node.create_subscription(
        sensor_msgs.msg.CameraInfo,
        '/camera_info',
        lambda msg: set_camera_info(msg),
        qos_profile_sensor_data
    )

    logger.info('rune node started')
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
