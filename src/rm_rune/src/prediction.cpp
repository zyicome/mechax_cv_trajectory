#include "prediction.hpp"
#include <chrono>
#include <cmath>
#include <vector>
#include <iostream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <sstream>

// 计算能量机关旋转的位置（弧度制）
double Prediction::radian(double time, double k, double b, double a, double omega, double phi) 
{
    // 计算能量机关的旋转位置，使用正弦函数来模拟旋转中的振荡部分
    return k * time + b + a * sin(omega * time + phi);
}

/* PRIVATE
计算旋转到的位置对应的角度，范围为 [0, 2pi/5)。
@param orientation: 表示方向的二维向量
@return: 返回计算的角度，范围在 [0, 2pi/5) 内
*/
double Prediction::angle_of(double angle_raw) {
    if(angle_raw < 0){angle_raw += M_PI*2;}
    // std::cout << "angle:" <<angle*180/M_PI<<std::endl;
    double mod = fmod(angle_raw, 2 * M_PI / 5);
    return angle_raw;
}

// RotationParams 默认构造函数，初始化旋转参数
RotationParams::RotationParams() 
{
    // 初始化默认参数
    k = 0;
    b = 0;
    a = 0.4;        // 振幅
    omega = 1.9;    // 圆速度
    phi = 0;        // 初相位
}

// RotationParams 带参构造函数，允许用户传入自定义参数
RotationParams::RotationParams(double k, double b, double a, double omega, double phi) 
{
    // 初始化参数
    this->k = k;
    this->b = b;
    this->a = a;
    this->omega = omega;
    this->phi = phi;
}

std::string RotationParams::to_string() const {
    std::ostringstream oss;
    oss << "k: " << k << ", b: " << b << ", a: " << a << ", omega: " << omega << ", phi: " << phi;
    return oss.str();
}

// 打包旋转参数到元组
tuple<double, double, double, double, double> RotationParams::pack() 
{
    // 返回旋转参数的元组
    return make_tuple(k, b, a, omega, phi);
}

// Prediction 构造函数，初始化定时器和旋转参数
Prediction::Prediction() 
{
    start_time = system_clock::now();         // 初始化开始时间
    last_fit_time = system_clock::now();      // 上次拟合时间
    last_update_time = system_clock::now();   // 上次更新时间
    params = RotationParams();                // 初始化旋转参数
}

// 检查当前时间与上次更新的时间差是否在合理范围内
bool Prediction::check_timeliness(system_clock::time_point current_time) {
    duration<double> elapsed_seconds = current_time - last_update_time;
    return elapsed_seconds.count() < 0.3;  // 如果时间差小于 0.3 秒，则认为是及时的
}

std::vector<double> Prediction::unwrapped_radians(std::vector<double> radians) {
    // 实现 np.unwrap(radians * 5) / 5 的逻辑
    if (radians.empty()) return {};

    std::vector<double> unwrapped;
    unwrapped.reserve(radians.size());

    // 将弧度乘以 5
    double scale = 5.0;
    double last = radians[0] * scale;
    unwrapped.push_back(last);

    for (size_t i = 1; i < radians.size(); ++i) {
        double current = radians[i] * scale;
        double delta = current - last;

        if (delta > M_PI) {
            delta -= 2 * M_PI;
        }
        else if (delta < -M_PI) {
            delta += 2 * M_PI;
        }

        double new_angle = last + delta;
        unwrapped.push_back(new_angle);
        last = new_angle;
    }

    // 将结果除以 5
    for(auto& angle : unwrapped){
        angle /= scale;
    }

    return unwrapped;
}


std::vector<double> Prediction::unwrapped_radians_yaw(std::vector<double> radians) {
    // 实现 np.unwrap(radians * 5) / 5 的逻辑
    if (radians.empty()) return {};

    std::vector<double> unwrapped;
    unwrapped.reserve(radians.size());

    double scale = 1.0;
    double last = radians[0] * scale;
    unwrapped.push_back(last);

    for (size_t i = 1; i < radians.size(); ++i) {
        double current = radians[i] * scale;
        double delta = current - last;

        if (delta > M_PI) {
            delta -= 2 * M_PI;
        }
        else if (delta < -M_PI) {
            delta += 2 * M_PI;
        }

        double new_angle = last + delta;
        unwrapped.push_back(new_angle);
        last = new_angle;
    }

    // 将结果除以 5
    for(auto& angle : unwrapped){
        angle /= scale;
    }

    return unwrapped;
}


// 检查是否满足拟合条件（弧度数量超过）
bool Prediction::can_fit() 
{
    bool result = radians.size() > 10;
    return result;
}

// 判断是否需要重新拟合旋转参数
bool Prediction::need_fit() {
    duration<double> elapsed_seconds = system_clock::now() - last_fit_time;
    return elapsed_seconds.count() > cfg.refit_delay_sec;  // 如果超过 refit_delay_sec 秒，则需要重新拟合
}

// 定义残差结构体
struct RotationResidual {
    RotationResidual(double t, double theta)
        : t_(t), theta_(theta) {}

    template <typename T>
    bool operator()(const T* const params, T* residual) const {
        // params = [k, b, a, omega, phi]
        T k = params[0];
        T b = params[1];
        T a = params[2];
        T omega = params[3];
        T phi = params[4];

        // 计算预测值
        T predicted = k * T(t_) + b + a * ceres::sin(omega * T(t_) + phi);
        residual[0] = T(theta_) - predicted;  // 残差 = 观测值 - 预测值
        return true;
    }

private:
    const double t_;
    const double theta_;
};

// 拟合旋转参数
void Prediction::fit() {
    if (radians.size() < 10) {  // 至少需要50个数据点
        std::cerr << "数据不足以进行拟合。" << std::endl;
        return;
    }

    // 获取解包后的弧度值
    std::vector<double> unwrapped = unwrapped_radians(radians);
    if (unwrapped.empty()) {
        std::cerr << "没有数据可用于拟合。" << std::endl;
        return;
    }

    // 准备初始猜测值
    double k_initial = params.k;
    double b_initial = params.b;
    double a_initial = params.a;
    double omega_initial = params.omega;
    double phi_initial = params.phi;

    double initial_params[5] = { k_initial, b_initial, a_initial, omega_initial, phi_initial };

    // 构建优化问题
    ceres::Problem problem;

    for (size_t i = 0; i < times_sec.size(); ++i) {
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<RotationResidual, 1, 5>(
                new RotationResidual(times_sec[i], unwrapped[i]));

        problem.AddResidualBlock(cost_function, nullptr, initial_params);
    }

    // 配置优化器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 检查拟合是否成功
    if (summary.termination_type == ceres::CONVERGENCE) {
        // 更新旋转参数
        params.k = initial_params[0];
        params.b = initial_params[1];
        params.a = initial_params[2];
        params.omega = initial_params[3];
        params.phi = initial_params[4];

        has_fitted = true;
        last_fit_time = std::chrono::system_clock::now();

        // 可选：输出拟合结果
        // std::cout << "拟合成功: " << params.to_string() << std::endl;
    } else {
        // 拟合失败
        // std::cerr << "拟合失败: " << summary.BriefReport() << std::endl;
    }
}

// 快速估计旋转的方向
bool Prediction::fast_estimate_sense_of_rotation() {
    if (radians_raw.empty()) return true;  // 默认正转

    double end = radians_raw.back();
    double start = radians_raw.front();
    return end > start;  // 如果末尾的角度大于开始的角度，则方向为正
}

// 根据时间和拟合的旋转参数预测旋转的位置
double Prediction::predict() {
    if (can_fit() && need_fit()) {
        fit();  // 如果可以拟合且需要拟合，则进行拟合
    }

    // 不使用拟合
    // has_fitted = false;

    if (!has_fitted) {
        // 如果未拟合，则使用快速估算来预测角度
        std::cout << "未拟合，使用快速估算" << std::endl;
        return fast_estimate_sense_of_rotation() ? (M_PI / 3.0 * cfg.hit_delay_sec) : (-M_PI / 3.0 * cfg.hit_delay_sec);
    }

    // 使用拟合的参数进行精确预测
    double current = radian(times_sec.back(), params.k, params.b, params.a, params.omega, params.phi);
    double predicted = radian(times_sec.back() + cfg.hit_delay_sec, params.k, params.b, params.a, params.omega, params.phi);
    if (fast_estimate_sense_of_rotation()){
        return abs(predicted - current);
    }
    else{
        return -abs(predicted - current);
    }
    // return predicted - current ;  // 返回预测的角度差
}

// 更新预测器的状态，记录当前时间和弧度
void Prediction::update(cv::Point2f orientation) {
    auto current_time = system_clock::now();
    if (!check_timeliness(current_time)) {
        reset();  // 如果更新不及时，则重置预测器
    }

    // if(!radians_raw.empty()){
    //     if (radians_raw.back() - atan2(orientation.y, orientation.x) > M_PI / 6) 
    //     {   
    //         radians.clear();  // 清空原始弧度数据
    //         reset();  
    //         std::cout << "reset prediction" << std::endl;
    //     }

    // }

    // radians_raw.push_back(atan2(orientation.y, orientation.x));
    // 记录当前的角度（弧度）和时间
    radians_raw.push_back(atan2(orientation.y, orientation.x));
    radians_raw = unwrapped_radians_yaw(radians_raw);
    
    // std::cout << "radians_raw: ";
    // for (const auto& radian : radians_raw) {
    //     std::cout << radian << " ";
    // }
    // std::cout << std::endl;




    for (auto& radian : radians_raw) {
        radians.push_back(angle_of(radian));
    }
    duration<double> elapsed_seconds = current_time - start_time;
    times_sec.push_back(elapsed_seconds.count());
    last_update_time = current_time;  // 更新最后一次更新时间
}

// 重置预测器
void Prediction::reset() {
    *this = Prediction();  // 重置为一个新的 Prediction 对象
}