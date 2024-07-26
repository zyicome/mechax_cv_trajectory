from dataclasses import dataclass

from numpy import array
from numpy import ndarray


# 定义一个不可变的配置类，用于存储应用的配置信息
@dataclass
class Configuration:
    enable: bool  # 是否启用节点
    binary_threshold: float
    kernel_size: int  # 形态学的核大小
    min_area: float  # 轮廓最小面积
    ref_hu: ndarray  # 参考HU值
    hu_dev_threshold: float  # HU值误差阈值
    target_relative_position: float  # 目标相对位置
    center_relative_position: float  # 中心相对位置
    refit_delay_sec: float  # 重新拟合延迟
    hit_delay_sec: float  # 命中延迟
    distance_correction_ratio: float  # 距离校正比例

    def __repr__(self):
        return f"""Configuration(
    enable={self.enable},
    binary_threshold={self.binary_threshold},
    kernel_size={self.kernel_size},
    min_area={self.min_area},
    ref_hu=array([
        {self.ref_hu[0]},
        {self.ref_hu[1]},
        {self.ref_hu[2]},
        {self.ref_hu[3]},
        {self.ref_hu[4]},
        {self.ref_hu[5]},
        {self.ref_hu[6]}
    ]),
    hu_dev_threshold={self.hu_dev_threshold},
    target_relative_position={self.target_relative_position},
    center_relative_position={self.center_relative_position},
    refit_delay_sec={self.refit_delay_sec},
    hit_delay_sec={self.hit_delay_sec},
    distance_correction_ratio={self.distance_correction_ratio}
)"""


cfg: Configuration = Configuration(
    enable=True,
    binary_threshold=254,
    kernel_size=15,
    min_area=7000,
    ref_hu=array([
        2.25365179e-01,
        1.46944335e-02,
        4.26202853e-03,
        1.52841796e-03,
        3.90094886e-06,
        1.85274967e-04,
        -8.84483057e-09
    ]),
    hu_dev_threshold=0.2,
    target_relative_position=0.6,
    center_relative_position=-3.2,
    refit_delay_sec=0.1,
    hit_delay_sec=0.5,
    distance_correction_ratio=1
)
