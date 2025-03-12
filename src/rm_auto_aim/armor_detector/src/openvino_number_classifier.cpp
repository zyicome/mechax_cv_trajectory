#include "armor_detector/openvino_number_classifier.hpp"

cv::Mat letterbox(const cv::Mat &input)
{
    int col = input.cols;
    int row = input.rows;
    int _max = std::max(col, row);
    cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
    input.copyTo(result(cv::Rect(0, 0, col, row)));
    return result;
}

double sigmoid(double x) {
        if(x>0)
            return 1.0 / (1.0 + exp(-x));
        else
            return exp(x) / (1.0 + exp(x));
    }

OpenvinoNumberClassifier::OpenvinoNumberClassifier(string model_path, string label_path, string device, double threshold, std::vector<std::string> & ignore_classes = {})
{
        // -------- Step 1. Initialize OpenVINO Runtime Core -------
        core_ = ov::Core();
        // -------- Step 2. Read a model --------
        std::shared_ptr<ov::Model> model;
        model = core_.read_model(model_path);
        if(model == nullptr)
        {
            std::cerr << "Model not loaded, check the model_path!" << std::endl;
            return;
        }
        // -------- Step 3. Compile a model --------
        compiled_model_ = core_.compile_model(model, device);
        // -------- Step 4. Create an Infer Request --------
        infer_request_ = compiled_model_.create_infer_request();
        std::cout << "Openvino onnx model loaded successfully!" << std::endl;

        std::ifstream label_file(label_path);
        std::string line;
        while (std::getline(label_file, line)) {
            class_names_.push_back(line);
        }
}

void OpenvinoNumberClassifier::extractNumbers(const cv::Mat & src, std::vector<Armor> & armors)
{
  // Light length in image
  const int light_length = 12;
  // Image size after warp
  const int warp_height = 28;
  const int small_armor_width = 32;
  const int large_armor_width = 54;
  // Number ROI size
  const cv::Size roi_size(20, 28);

  for (auto & armor : armors) {
    // Warp perspective transform
    cv::Point2f lights_vertices[4] = {
      armor.left_light.bottom, armor.left_light.top, armor.right_light.top,
      armor.right_light.bottom};

    const int top_light_y = (warp_height - light_length) / 2 - 1;
    const int bottom_light_y = top_light_y + light_length;
    const int warp_width = armor.type == ArmorType::SMALL ? small_armor_width : large_armor_width;
    cv::Point2f target_vertices[4] = {
      cv::Point(0, bottom_light_y),
      cv::Point(0, top_light_y),
      cv::Point(warp_width - 1, top_light_y),
      cv::Point(warp_width - 1, bottom_light_y),
    };
    cv::Mat number_image;
    auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
    cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

    // Get ROI
    number_image =
      number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

    // Binarize
    number_image = numberlcassfy_helper(number_image);
    cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
    cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    armor.number_img = number_image;
  }
}

void OpenvinoNumberClassifier::infer(std::vector<Armor> & armors)
{
    for (auto & armor : armors) {
        cv::Mat image = armor.number_img.clone();
        cv::Mat letterbox_img = letterbox(image);
        cv::Mat blob;
        cv::dnn::blobFromImage(letterbox_img, blob, 1.0 / 255.0, cv::Size(IMAGE_HEIGHT, IMAGE_WIDTH), cv::Scalar(), true, false);

        auto input_port = compiled_model_.input();
        ov::Tensor input_tensor(input_port.get_element_type(), input_port.get_shape(), blob.ptr(0));

        // -------- Step 6. Do inference --------
        infer_request_.set_input_tensor(input_tensor);
        infer_request_.infer();

        auto output0 = infer_request_.get_output_tensor(0);
        cv::Mat output_buffer(output0.get_shape()[1], output0.get_shape()[2], CV_32F, output0.data());
        // Do softmax
        float max_prob = *std::max_element(output_buffer.begin<float>(), output_buffer.end<float>());
        cv::Mat softmax_prob;
        cv::exp(output_buffer - max_prob, softmax_prob);
        float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
        softmax_prob /= sum;

        double confidence;
        cv::Point class_id_point;
        minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
        int label_id = class_id_point.x;

        armor.confidence = confidence;
        armor.number = class_names_[label_id];

        std::stringstream result_ss;
        result_ss << armor.number << ": " << std::fixed << std::setprecision(1)
                << armor.confidence * 100.0 << "%";
        armor.classfication_result = result_ss.str();
    }

    armors.erase(
    std::remove_if(
      armors.begin(), armors.end(),
      [this](const Armor & armor) {
        if (armor.confidence < threshold) {
          return true;
        }

        for (const auto & ignore_class : ignore_classes_) {
          if (armor.number == ignore_class) {
            return true;
          }
        }

        bool mismatch_armor_type = false;
        if (armor.type == ArmorType::LARGE) {
          mismatch_armor_type =
            armor.number == "outpost" || armor.number == "2" || armor.number == "guard";
        } else if (armor.type == ArmorType::SMALL) {
          mismatch_armor_type = armor.number == "1" || armor.number == "base";
        }
        return mismatch_armor_type;
      }),
    armors.end());
         
}

