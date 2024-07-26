from functools import cache

import cv2
import numpy
from cv2 import circle
from cv2 import COLOR_BGR2HSV
from cv2 import cvtColor
from cv2 import dilate
from cv2 import threshold
from numpy import zeros

from rune.configuration import cfg
from rune.GUI.listener import listen


@cache
def kernel(size: int):
    n = zeros((size, size), dtype='uint8')
    return circle(n, (size // 2, size // 2), size // 2, (1,), -1)


@listen
def to_gray(image: numpy.ndarray) -> numpy.ndarray:
    hsv = cvtColor(image, COLOR_BGR2HSV)
    gray = hsv[:, :, 2]
    gray = dilate(gray, kernel(cfg.kernel_size))
    return gray


def to_binary(image: numpy.ndarray) -> numpy.ndarray:
    _, binary = threshold(
        image,
        cfg.binary_threshold,  # 自动计算得出阈值
        255,
        cv2.THRESH_BINARY
        # + cv2.THRESH_TRIANGLE,
    )
    return binary
