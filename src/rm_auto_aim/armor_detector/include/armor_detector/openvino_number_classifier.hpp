#include <opencv2/opencv.hpp>
#include "openvino/openvino.hpp"
#include <vector>
#include <iostream>
#include <fstream>

#include "armor_detector/armor.hpp"

class OpenvinoNumberClassifier
{
public:
    const int IMAGE_HEIGHT = 20;
    const int IMAGE_WIDTH = 28;

    ov::Core core_;
    ov::CompiledModel compiled_model_;
    ov::InferRequest infer_request_;

    double threshold;

    std::vector<std::string> class_names_;

    OpenvinoNumberClassifier(string model_path, string label_path, string device, double threshold, std::vector<std::string> & ignore_classes = {});

    void extractNumbers(const cv::Mat & src, std::vector<Armor> & armors);

    void infer(std::vector<Armor> & armors);
}