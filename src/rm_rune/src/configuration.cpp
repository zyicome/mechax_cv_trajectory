#include "configuration.hpp"

Configuration::Configuration()
{
    init();
}

void Configuration::init()
{
    enable=true;
    binary_threshold=254;
    kernel_size=15;
    min_area=7000;
    double data[] = {2.25365179e-01, 1.46944335e-02, 4.26202853e-03, 1.52841796e-03, 3.90094886e-06, 1.85274967e-04, -8.84483057e-09};
    ref_hu = cv::Mat(1, 7, CV_64F, data);
    hu_dev_threshold=0.2;
    target_relative_position=0.6;
    center_relative_position=-3.2;
    refit_delay_sec=0.1;
    hit_delay_sec=0.5;
    distance_correction_ratio=1;
}