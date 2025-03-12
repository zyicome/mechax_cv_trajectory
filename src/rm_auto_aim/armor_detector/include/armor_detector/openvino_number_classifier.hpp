#include <opencv2/opencv.hpp>
#include "openvino/openvino.hpp"
#include <vector>
#include <iostream>
#include <fstream>

#include "armor_detector/armor.hpp"

namespace rm_auto_aim
{

class OpenvinoNumberClassifier
{
public:
    const int IMAGE_HEIGHT = 20;
    const int IMAGE_WIDTH = 28;

    ov::Core core_;
    ov::CompiledModel compiled_model_;
    ov::InferRequest infer_request_;

    double threshold_;

    std::vector<std::string> class_names_;
    std::vector<std::string> ignore_classes_;

    OpenvinoNumberClassifier(std::string model_path, std::string label_path, std::string device);

    void classifierSet(const double threshold, const std::vector<std::string> & ignore_classes = {});

    void extractNumbers(const cv::Mat & src, std::vector<Armor> & armors);

    void infer(std::vector<Armor> & armors);

    cv::Mat numberlcassfy_helper(cv::Mat & number_image);
    double get_weights_parameter(cv::Mat & number_image);
};

} // namespace rm_auto_aim