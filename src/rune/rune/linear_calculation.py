"""
利用线性代数计算图像的信息。
"""
import cv2
import numpy
from numpy import ndarray


def hu_moments_deviation(hu_moments: ndarray, reference: ndarray) -> float:
    """
    计算Hu矩偏差。

    @param hu_moments: 图像的Hu矩。
    @param reference: 参考Hu矩。

    @return: 计算得到的Hu矩偏差值。
    """
    # region Validation of input parameters
    if not isinstance(hu_moments, ndarray) or not isinstance(reference, ndarray):
        raise TypeError("输入必须是numpy数组。")
    if hu_moments.size != 7:
        raise ValueError(f"输入数组hu_moments的大小必须为7，实际是{hu_moments.size}。")
    if reference.size != 7:
        raise ValueError(f"输入数组reference的大小必须为7，实际是{reference.size}。")
    # endregion
    hu_moments = hu_moments.reshape((7,)).copy()
    reference = reference.reshape((7,)).copy()
    # 对数变换
    hu_moments = -numpy.log10(abs(hu_moments)) * numpy.sign(hu_moments)
    reference = -numpy.log10(abs(reference)) * numpy.sign(reference)
    return sum(abs(1/hu_moments[i] - 1/reference[i]) for i in range(6))


class ContourInfo:
    def __init__(self, contour: ndarray):
        self.contour = contour
        self.moments = cv2.moments(contour)
        self.hu_moments = cv2.HuMoments(self.moments)

        # region Validation of input parameters
        required_keys = ['m00', 'm10', 'm01', 'mu20', 'mu11', 'mu02', 'nu20', 'nu11', 'nu02']
        for key in required_keys:
            if key not in self.moments:
                raise ValueError(f"输入的轮廓的矩没有{key}。")
            if not isinstance(self.moments[key], float | int):
                raise TypeError(f"输入的轮廓的矩{key}的类型必须是float或int。")
        # endregion

    @property
    def area(self) -> float:
        return self.moments['m00']

    @property
    def centroid(self) -> ndarray:
        """
        计算图像的重心。
        @return: 计算得到的重心。
        """
        return numpy.array((
            self.moments['m10'] / self.moments['m00'],
            self.moments['m01'] / self.moments['m00']
        ))

    @property
    def eigen_max(self) -> tuple[ndarray, float]:
        """
        计算图像的特征向量和特征值。
        @return: 计算得到最大的特征向量和特征值。
        """
        # 二阶归一化中心矩
        nu_xx = self.moments['mu20'] / self.moments['m00']
        nu_yy = self.moments['mu02'] / self.moments['m00']
        nu_xy = self.moments['mu11'] / self.moments['m00']

        # 构建协方差矩阵
        cov_matrix = numpy.array((
            (nu_xx, nu_xy),
            (nu_xy, nu_yy)
        ))

        # 计算特征值与特征向量
        eigenvalues, eigenvectors = numpy.linalg.eig(cov_matrix)

        # 获取最大特征值对应的特征向量（主轴方向）
        max_eigenvalue_index = numpy.argmax(eigenvalues)
        major_axis_vector = eigenvectors[:, max_eigenvalue_index]
        variance = eigenvalues[max_eigenvalue_index]

        return major_axis_vector, variance

    @property
    def skewness(self) -> ndarray:
        """
        计算图像的偏度。
        @return: 计算得到的偏度。
        """
        return numpy.array([
            self.moments['mu30'] / self.moments['m00'],
            self.moments['mu03'] / self.moments['m00']
        ])
