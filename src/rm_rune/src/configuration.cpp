// 需修改
// 调参函数


#include "configuration.hpp"

// Configuration 构造函数，调用 init 函数初始化成员变量
Configuration::Configuration()
{
    init();  // 调用初始化函数
}

// init 函数，用于初始化 Configuration 对象的默认参数
void Configuration::init()
{
    // 启用开关，默认为 true
    enable = true;
    
    // 二值化阈值设置，用于图像处理中的阈值分割
    binary_threshold = 254;
    
    // 内核大小，用于图像形态学操作（如腐蚀、膨胀等），默认为 15
    kernel_size = 15;
    
    // 最小轮廓面积，用于过滤面积过小的轮廓
    min_area = 7000;

    // Hu 矩阵的参考数据，用于形状匹配
    // Hu 矩阵是图像中常用于表示形状特征的七个不变矩
    double data[] = {2.25365179e-01, 1.46944335e-02, 4.26202853e-03, 1.52841796e-03, 3.90094886e-06, 1.85274967e-04, -8.84483057e-09};
    
    // 创建一个 1x7 的 cv::Mat 矩阵，并使用给定的 Hu 矩阵数据初始化
    ref_hu = cv::Mat(1, 7, CV_64F, data);
    
    // Hu 矩阵匹配时的偏差阈值，用于判断形状相似性，阈值为 0.2
    hu_dev_threshold = 0.2;

    // 目标相对位置系数，用于计算目标相对于质心的位置，默认为 0.6
    target_relative_position = 0.6;
    
    // 中心相对位置系数，用于计算中心相对于质心的位置，默认为 -3.2
    center_relative_position = -3.2;
    
    // 重定位延迟时间（秒），在系统需要重新定位时的时间延迟，默认为 0.1 秒
    refit_delay_sec = 0.1;
    
    // 击中延迟时间（秒），在系统检测到击中时的延迟时间，默认为 0.5 秒
    hit_delay_sec = 0.5;
    
    // 距离校正比例，用于对目标距离进行校正，默认为 1（无校正）
    distance_correction_ratio = 1;
}
