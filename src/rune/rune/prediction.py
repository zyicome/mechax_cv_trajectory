import datetime
from dataclasses import dataclass

import numpy
import numpy as np

from datetime import datetime

from rune.configuration import cfg
from scipy.optimize import curve_fit

from rune.logging import logger


def radian(time: float | np.ndarray, k: float, b: float, a: float, omega: float, phi: float) -> float | np.ndarray:
    r"""
    能量机关旋转的位置（弧度制）
    $$
    \theta = k t + b + a \cos \left( \omega t + \phi \right)
    $$

    @param time: 时间，可以是单个数字或一个数组
    @param k: 斜率
    @param b: 截距
    @param a: 振幅
    @param omega: 圆速度
    @param phi: 初相位
    @return: 旋转到的位置
    """
    return k * time + b + a * np.sin(omega * time + phi)


def angle_of(orientation: np.ndarray):
    """
    计算旋转到的位置对应的角度
    @param orientation: 旋转到的位置
    @return: 角度,[0, 2pi/5)
    """
    # 能量机关转动2pi/5后循环
    return np.remainder(np.arctan2(orientation[1], orientation[0]), 2 * np.pi / 5)


@dataclass
class RotationParams:
    k: float = 0
    b: float = 0
    a: float = 0.4
    omega: float = 1.9
    phi: float = 0

    def pack(self) -> tuple[float, float, float, float, float]:
        return self.k, self.b, self.a, self.omega, self.phi

    def __str__(self):
        return f"参数[k:{self.k:>5.3f} b:{self.b:>5.3f} a:{self.a:>5.3f} omega:{self.omega:>5.3f} phi:{self.phi:>5.3f}]"


class Predictor:
    def __init__(self):
        self.start_time: datetime = datetime.now()
        self.last_fit_time: datetime = datetime.now()
        self.last_update_time: datetime = datetime.now()
        self.params: RotationParams = RotationParams()
        self.has_fitted: bool = False
        self.radians: list[float] = []
        self.times_sec: list[float] = []

    def check_timeliness(self, current_time: datetime):
        """
        检查预测信息是否具有时效性
        @param current_time: 当前时间
        @return: 是否有效
        """
        return (current_time - self.last_update_time).total_seconds() < 0.3

    @property
    def unwrapped_radians(self):
        return np.unwrap(np.array(self.radians) * 5) / 5

    def can_fit(self):
        """
        检查是否可以进行拟合
        @return: 是否可以进行拟合
        """
        return len(self.radians) > 50

    def need_fit(self):
        """
        检查是否需要进行拟合
        @return: 是否需要进行拟合
        """
        return (datetime.now() - self.last_fit_time).total_seconds() > cfg.refit_delay_sec

    def fit(self):
        try:
            # noinspection PyTupleAssignmentBalance
            params, _ = curve_fit(radian, self.times_sec, self.unwrapped_radians, p0=self.params.pack())
            self.params = RotationParams(*params)
            self.last_fit_time = datetime.now()
            self.has_fitted = True
        except Exception as e:
            logger.warning(f"拟合失败：{e}")

    def fast_estimate_sense_of_rotation(self) -> bool:
        """
        当数据过少时，判断旋转方向
        @return: 是否正转
        """
        radians = self.unwrapped_radians
        end: float = float(radians[-1])
        start: float = float(radians[0])
        return end > start

    def predict(self) -> float:
        """
        预测需要旋转的角度
        @return: 旋转到的位置
        """
        if self.can_fit() and self.need_fit():
            self.fit()

        if not self.has_fitted:
            if self.fast_estimate_sense_of_rotation():
                return np.pi / 3 * cfg.hit_delay_sec
            return -np.pi / 3 * cfg.hit_delay_sec

        current = radian(self.times_sec[-1], *self.params.pack())
        predicted = radian(self.times_sec[-1] + cfg.hit_delay_sec, *self.params.pack())
        return predicted - current

    def update(self, orientation: np.ndarray):
        """
        更新预测信息
        @param orientation: 旋转到的位置
        """
        current_time = datetime.now()
        if not self.check_timeliness(current_time):
            self.reset()

        self.radians.append(angle_of(orientation))
        self.times_sec.append((current_time - self.start_time).total_seconds())
        self.last_update_time = current_time

    def reset(self):
        self.__init__()
