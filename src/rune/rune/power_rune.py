from math import cos, sin
from math import sqrt

import numpy

from rune.configuration import cfg
from rune.GUI.listener import listen
from rune.linear_calculation import ContourInfo
from rune.linear_calculation import hu_moments_deviation


def hu_condition(contour: ContourInfo):
    return hu_moments_deviation(contour.hu_moments, cfg.ref_hu) <= cfg.hu_dev_threshold


def area_condition(contour: ContourInfo):
    return contour.moments['m00'] > cfg.min_area


@listen
def filter_by_area(contours: list[ContourInfo]):
    return list(filter(area_condition, contours))


@listen
def filter_by_hu(contours: list[ContourInfo]):
    filtered = (
        (contour, deviation)
        for contour in contours
        for deviation in (hu_moments_deviation(contour.hu_moments, cfg.ref_hu),)
        if deviation <= cfg.hu_dev_threshold
    )
    return [x[0] for x in sorted(filtered, key=lambda x: x[1])]


class Blade:
    """
    @ivar contour: 输入的轮廓信息。
    @ivar target: 根据特征向量和配置中的目标相对位置计算的目标位置。
    @ivar center: 根据特征向量和配置中的中心相对位置计算的中心位置。
    @ivar vector: 根据目标位置和中心位置计算出的向量。
    @param contour: ContourInfo类型的实例。
    """

    def __init__(self, contour: ContourInfo):
        centroid = contour.centroid
        axis, value = contour.eigen_max
        skewness = contour.skewness
        # 根据主轴和偏度计算方向，如果内积为负，则方向为原主轴的反方向
        orientation = axis if axis @ skewness < 0 else - axis
        trait = orientation * sqrt(value)

        self.contour = contour
        # 根据特征向量和配置文件中的相对位置，计算目标位置和中心位置
        self.target = centroid + trait * cfg.target_relative_position
        self.center = centroid + trait * cfg.center_relative_position
        self.vector = self.target - self.center


@listen
def predict(vector: numpy.ndarray, radian: float):
    return numpy.array([[cos(radian), -sin(radian)], [sin(radian), cos(radian)]]) @ vector
