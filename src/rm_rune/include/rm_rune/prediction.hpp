#pragma once

#include <chrono>
#include <cmath>
#include <vector>
#include <iostream>
#include <tuple>
#include <string>
#include <sstream>

#include "configuration.hpp"          // 假设配置设置在这里定义
#include <ceres/ceres.h>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std;
using namespace std::chrono;

/**
 * @brief 旋转参数类，用于存储和管理旋转模型的参数。
 */
class RotationParams
{
public:
    /**
     * @brief 默认构造函数，初始化旋转参数。
     */
    RotationParams();

    /**
     * @brief 带参数的构造函数，允许用户自定义旋转参数。
     * @param k 斜率
     * @param b 截距
     * @param a 振幅
     * @param omega 圆速度
     * @param phi 初相位
     */
    RotationParams(double k, double b, double a, double omega, double phi);

    /**
     * @brief 将旋转参数打包成元组。
     * @return 包含旋转参数的元组 (k, b, a, omega, phi)。
     */
    tuple<double, double, double, double, double> pack();

    /**
     * @brief 获取旋转参数的字符串表示。
     * @return 字符串格式的旋转参数。
     */
    std::string to_string() const;

    // 旋转模型参数
    double k;      ///< 斜率
    double b;      ///< 截距
    double a;      ///< 振幅
    double omega;  ///< 圆速度
    double phi;    ///< 初相位
};

/**
 * @brief 预测类，用于管理和预测旋转位置。
 */
class Prediction
{
public:
    /**
     * @brief 构造函数，初始化定时器和旋转参数。
     */
    Prediction();

    /**
     * @brief 检查预测信息是否具有时效性。
     * @param current_time 当前时间点。
     * @return 如果预测信息在有效时间范围内返回 true，否则返回 false。
     */
    bool check_timeliness(system_clock::time_point current_time);

    /**
     * @brief 获取解包后的弧度值，确保角度的连续性。
     * @return 解包后的弧度值向量。
     */
    vector<double> unwrapped_radians(vector<double> radians);
    vector<double> unwrapped_radians_yaw(vector<double> radians);


    /**
     * @brief 检查是否满足拟合条件（弧度数量是否超过50）。
     * @return 如果可以进行拟合返回 true，否则返回 false。
     */
    bool can_fit();

    /**
     * @brief 判断是否需要重新拟合旋转参数。
     * @return 如果需要重新拟合返回 true，否则返回 false。
     */
    bool need_fit();

    /**
     * @brief 执行拟合操作，使用 Ceres Solver 进行非线性曲线拟合。
     */
    void fit();

    /**
     * @brief 快速估计旋转的方向（正转或反转），当数据过少时使用。
     * @return 如果旋转方向为正转返回 true，否则返回 false。
     */
    bool fast_estimate_sense_of_rotation();

    /**
     * @brief 根据当前数据预测需要旋转的角度。
     * @return 预测需要旋转的角度（弧度）。
     */
    double predict();

    /**
     * @brief 更新预测器的状态，记录当前时间和弧度。
     * @param orientation 当前的方向向量。
     */
    void update(cv::Point2f orientation);

    /**
     * @brief 重置预测器，清空所有记录的数据和状态。
     */
    void reset();

public:
    // 时间相关成员变量
    system_clock::time_point start_time;       ///< 开始时间
    system_clock::time_point last_fit_time;    ///< 上次拟合时间
    system_clock::time_point last_update_time; ///< 上次更新时间

    // 旋转参数和配置
    RotationParams params;      ///< 当前的旋转参数
    Configuration cfg;          ///< 配置参数

    // 拟合状态和数据记录
    bool has_fitted = false;    ///< 是否已经进行过拟合
    vector<double> radians;     ///< 记录的弧度值
    vector<double> radians_raw;     // 原始记录的弧度值
    vector<double> times_sec;   ///< 记录的时间（秒）

private:
    /**
     * @brief 计算能量机关旋转的位置（弧度制）。
     * @param time 时间，可以是单个数字或一个数组。
     * @param k 斜率。
     * @param b 截距。
     * @param a 振幅。
     * @param omega 圆速度。
     * @param phi 初相位。
     * @return 旋转到的位置（弧度制）。
     */
    double radian(double time, double k, double b, double a, double omega, double phi);

    /**
     * @brief 计算旋转到的位置对应的角度，范围为 [0, 2pi/5)。
     * @param orientation 表示方向的二维向量。
     * @return 返回计算的角度，范围在 [0, 2pi/5) 内。
     */
    double angle_of(double angle_raw);
};