#include "prediction.hpp"

   /*能量机关旋转的位置（弧度制）
    $$
    \theta = k t + b + a \cos \left( \omega t + \phi \right)
    $$

    @param time: 时间，可以是单个数字或一个数组
    @param k: 斜率
    @param b: 截距
    @param a: 振幅
    @param omega: 圆速度
    @param phi: 初相位
    @return: 旋转到的位置*/
double radian(double time, double k, double b, double a, double omega, double phi) 
{
    // 能量机关旋转的位置（弧度制）
    return k * time + b + a * sin(omega * time + phi);
}

    /*计算旋转到的位置对应的角度
    @param orientation: 旋转到的位置
    @return: 角度,[0, 2pi/5)*/
//能量机关转动2pi/5后循环
double angle_of(cv::Point2f orientation) {
    // 计算旋转到的位置对应的角度
    // Assuming orientation is a 2x1 Mat with float type
    float x = orientation.x;
    float y = orientation.y;
    return fmod(atan2(y, x), 2 * M_PI / 5);
}

RotationParams::RotationParams() 
{
    k = 0;
    b = 0;
    a = 0.4;
    omega = 1.9;
    phi = 0;
}

RotationParams::RotationParams(double k, double b, double a, double omega, double phi) 
{
    this->k = k;
    this->b = b;
    this->a = a;
    this->omega = omega;
    this->phi = phi;
}

tuple<double, double, double, double, double> RotationParams::pack() 
{
    return make_tuple(k, b, a, omega, phi);
}

Prediction::Prediction() 
{
    start_time = system_clock::now();
    last_fit_time = system_clock::now();
    last_update_time = system_clock::now();
    params = RotationParams();
}

bool Prediction::check_timeliness(system_clock::time_point current_time) {
        duration<double> elapsed_seconds = current_time - last_update_time;
        return elapsed_seconds.count() < 0.3;
}

//????????
vector<double> Prediction::unwrapped_radians() {
        // Unwrapping radians, similar to numpy.unwrap
        // This is a simplified version, actual implementation may vary
        vector<double> unwrapped = radians; // Placeholder for actual unwrapping logic
        return unwrapped;
    }

bool Prediction::can_fit() 
{
    bool result = radians.size() > 50;
    return result;
}

bool Prediction::need_fit() {
        duration<double> elapsed_seconds = system_clock::now() - last_fit_time;
        return elapsed_seconds.count() > cfg.refit_delay_sec; // Assuming cfg.refit_delay_sec is defined
}

//???????
void Prediction::fit() {
        // Fitting the rotation parameters
        // This is a placeholder for actual fitting logic
        params = RotationParams(); // Placeholder for actual fitting logic
        has_fitted = true;
        last_fit_time = system_clock::now();
    }

bool Prediction::fast_estimate_sense_of_rotation() {
        double end = radians.back();
        double start = radians.front();
        return end > start;
    }

double Prediction::predict() {
    if (can_fit() && need_fit()) {
        fit();
    }

    if (!has_fitted) {
        return fast_estimate_sense_of_rotation() ? M_PI / 3 * cfg.hit_delay_sec : -M_PI / 3 * cfg.hit_delay_sec;
    }

    double current = radian(times_sec.back(), params.k, params.b, params.a, params.omega, params.phi);
    double predicted = radian(times_sec.back() + cfg.hit_delay_sec, params.k, params.b, params.a, params.omega, params.phi);
    return predicted - current;
    }

void Prediction::update(cv::Point2f orientation) {
    auto current_time = system_clock::now();
    if (!check_timeliness(current_time)) {
        reset();
    }

    radians.push_back(angle_of(orientation));
    duration<double> elapsed_seconds = current_time - start_time;
    times_sec.push_back(elapsed_seconds.count());
    last_update_time = current_time;
}

void Prediction::reset() {
    *this = Prediction();
}