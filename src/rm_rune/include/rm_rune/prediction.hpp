#pragma once
#include <chrono>
#include <cmath>
#include <vector>
#include <iostream>
#include "configuration.hpp" // Assume configuration settings are defined here

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std;
using namespace std::chrono;

class RotationParams
{
public:
    RotationParams();
    RotationParams(double k, double b, double a, double omega, double phi);
    tuple<double, double, double, double, double> pack();

    double k;
    double b;
    double a;
    double omega;
    double phi;
};

class Prediction
{
public:
    Prediction();
    bool check_timeliness(system_clock::time_point current_time);
    vector<double> unwrapped_radians();
    bool can_fit();
    bool need_fit();
    void fit();
    bool fast_estimate_sense_of_rotation();
    double predict();
    void update(cv::Point2f orientation);
    void reset();

    system_clock::time_point start_time;
    system_clock::time_point last_fit_time;
    system_clock::time_point last_update_time;
    RotationParams params;
    Configuration cfg;
    bool has_fitted = false;
    vector<double> radians;
    vector<double> times_sec;
};