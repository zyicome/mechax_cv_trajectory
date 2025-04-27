#include "openvino_detect.hpp"  // 引入自定义头文件
#include <memory>       // 智能指针
#include <opencv2/dnn.hpp> // OpenCV DNN模块
#include <random>       // 随机数生成

namespace yolo { // 命名空间yolo

// 构造函数1: 使用默认输入尺寸(640x640)初始化模型
Inference::Inference(const std::string &model_path, 
                    const float &model_confidence_threshold,
                    const float &model_NMS_threshold) {
    model_input_shape_ = cv::Size(640, 640);   // 设置默认输入尺寸
    model_confidence_threshold_ = model_confidence_threshold; // 置信度阈值
    model_NMS_threshold_ = model_NMS_threshold; // NMS阈值
    InitializeModel(model_path); // 调用模型初始化方法
}

// 构造函数2: 使用指定输入尺寸初始化模型
Inference::Inference(const std::string &model_path,
                    const cv::Size model_input_shape,
                    const float &model_confidence_threshold,
                    const float &model_NMS_threshold) {
    model_input_shape_ = model_input_shape;     // 设置指定输入尺寸
    model_confidence_threshold_ = model_confidence_threshold;
    model_NMS_threshold_ = model_NMS_threshold;
    InitializeModel(model_path);
}

// 模型初始化方法
void Inference::InitializeModel(const std::string &model_path) {
    ov::Core core; // 创建OpenVINO核心对象
    // 读取模型文件
    std::shared_ptr<ov::Model> model = core.read_model(model_path);

    // 处理动态形状模型: 将输入形状固定为指定尺寸
    if (model->is_dynamic()) {
        model->reshape({1, 3, 
                       static_cast<long int>(model_input_shape_.height),
                       static_cast<long int>(model_input_shape_.width)});
    }

    // 创建预处理管道
    ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);
    // 输入张量设置: 数据类型U8, 布局NHWC, BGR颜色格式
    ppp.input().tensor()
        .set_element_type(ov::element::u8)
        .set_layout("NHWC")
        .set_color_format(ov::preprocess::ColorFormat::BGR);
    // 预处理步骤: 转换为FP32，转RGB颜色空间，归一化(除以255)
    ppp.input().preprocess()
        .convert_element_type(ov::element::f32)
        .convert_color(ov::preprocess::ColorFormat::RGB)
        .scale({255, 255, 255});
    // 模型输入布局设置为NCHW
    ppp.input().model().set_layout("NCHW");
    // 输出张量设置为FP32
    ppp.output().tensor().set_element_type(ov::element::f32);
    model = ppp.build(); // 完成预处理管道构建

    // 编译模型，选择设备
    compiled_model_ = core.compile_model(model, "AUTO");
    // 创建推理请求对象
    inference_request_ = compiled_model_.create_infer_request();

    // 获取模型输入尺寸
    const std::vector<ov::Output<ov::Node>> inputs = model->inputs();
    const ov::Shape input_shape = inputs[0].get_shape();
    model_input_shape_ = cv::Size(input_shape[2], input_shape[1]); // 宽x高

    // 获取模型输出尺寸
    const std::vector<ov::Output<ov::Node>> outputs = model->outputs();
    const ov::Shape output_shape = outputs[0].get_shape();
    model_output_shape_ = cv::Size(output_shape[2], output_shape[1]);
}

// 执行推理流程
void Inference::RunInference(cv::Mat &frame) {
    contours.clear(); // 清空轮廓数据
    Preprocessing(frame);   // 预处理
    inference_request_.infer(); // 执行推理
    PostProcessing(frame);  // 后处理
    // cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);

}

// 图像预处理方法
void Inference::Preprocessing(const cv::Mat &frame) {
    cv::Mat resized_frame;
    // 将输入图像缩放到模型输入尺寸(双线性插值)
    cv::resize(frame, resized_frame, model_input_shape_, 0, 0, cv::INTER_LINEAR);

    // 计算缩放比例(用于后续坐标还原)
    scale_factor_.x = static_cast<float>(frame.cols) / model_input_shape_.width;
    scale_factor_.y = static_cast<float>(frame.rows) / model_input_shape_.height;


    // 创建OpenVINO输入张量
    const ov::Tensor input_tensor = ov::Tensor(
        compiled_model_.input().get_element_type(),  // 获取模型输入数据类型
        compiled_model_.input().get_shape(),         // 获取模型输入形状
        (float*)resized_frame.data                   // 图像数据指针
    );
    inference_request_.set_input_tensor(input_tensor); // 设置输入张量
}

// 推理后处理方法
void Inference::PostProcessing(cv::Mat &frame) {
    std::vector<int> class_list;    // 存储类别ID
    std::vector<float> confidence_list; // 存储置信度
    std::vector<cv::Point2f> keypoints; // 存储关键点坐标
    std::vector<std::vector<cv::Point2f>> key_points_vec; 
    std::vector<cv::Rect> box_list; // 存储边界框

    // 获取输出张量数据指针
    const float* detections = inference_request_.get_output_tensor().data<const float>();
    // 将输出数据转换为OpenCV矩阵
    cv::Mat detection_outputs(model_output_shape_, CV_32F, (float*)detections);

    //8400x23 detection_outputs

    // std::cout << "detection_outputs: " << detection_outputs.cols << std::endl;
    // std::cout << "detection_outputs: " << detection_outputs.rows << std::endl;
    // std::cout << "classes_.size(): " << classes_.size() << std::endl;


    // 遍历所有检测结果
    for (int i = 0; i < detection_outputs.cols; ++i) {
        // 提取类别概率部分(第4行开始)
        cv::Mat classes_scores = detection_outputs.col(i).rowRange(4, classes_.size()+4);
        
        cv::Point class_id; // 最高概率类别ID
        double score;       // 最高概率值
        // 寻找最大概率值和对应类别
        cv::minMaxLoc(classes_scores, nullptr, &score, nullptr, &class_id);

        // 过滤低置信度检测结果
        if (score > model_confidence_threshold_) {
            class_list.push_back(class_id.y);     // 记录类别ID
            confidence_list.push_back(score);     // 记录置信度
            
            // 提取边界框坐标(x, y为中心点坐标，w,h为宽高)
            const float x = detection_outputs.at<float>(0, i);
            const float y = detection_outputs.at<float>(1, i);
            const float w = detection_outputs.at<float>(2, i);
            const float h = detection_outputs.at<float>(3, i);

            const float x1 = detection_outputs.at<float>(8, i);
            const float y1 = detection_outputs.at<float>(9, i);
            const float x2 = detection_outputs.at<float>(11, i);
            const float y2 = detection_outputs.at<float>(12, i);
            const float x3 = detection_outputs.at<float>(14, i);
            const float y3 = detection_outputs.at<float>(15, i);
            const float x4 = detection_outputs.at<float>(17, i);
            const float y4 = detection_outputs.at<float>(18, i);
            const float x5 = detection_outputs.at<float>(20, i);
            const float y5 = detection_outputs.at<float>(21, i); 

            keypoints.push_back(cv::Point2f(x1, y1)); // 记录关键点坐标
            keypoints.push_back(cv::Point2f(x2, y2));
            keypoints.push_back(cv::Point2f(x3, y3));
            keypoints.push_back(cv::Point2f(x4, y4));
            keypoints.push_back(cv::Point2f(x5, y5));

            key_points_vec.push_back(keypoints); // 存储关键点坐标
            keypoints.clear(); // 清空关键点坐标

            // 计算边界框坐标(转换为左上角坐标)
            cv::Rect box;
            box.x = static_cast<int>(x - w / 2);  // 左上角x坐标
            box.y = static_cast<int>(y - h / 2);  // 左上角y坐标
            box.width = static_cast<int>(w);      // 宽度
            box.height = static_cast<int>(h);     // 高度
            box_list.push_back(box);
        }
    }

    // std::cout << "debug" << std::endl;
    
    // 执行非极大值抑制(NMS)过滤重叠框
    std::vector<int> NMS_result;
    cv::dnn::NMSBoxes(box_list, confidence_list, 
                      model_confidence_threshold_, 
                      model_NMS_threshold_,
                      NMS_result);

    // 处理NMS后的检测结果
    for (size_t i = 0; i < NMS_result.size(); ++i) {
        Detection result;
        const int id = NMS_result[i]; // 获取保留结果的索引
        // 记录检测结果
        result.key_points = GetBoundingkeypoints(key_points_vec[id]); // 关键点坐标
        // std::cout << "key_points: " << result.key_points.size() << std::endl;
        result.class_id = class_list[id];     // 类别ID
        result.confidence = confidence_list[id]; // 置信度
        result.box = GetBoundingBox(box_list[id]); // 缩放后的边界框


        contours.emplace_back();
        contours.back().push_back(cv::Point2f(result.class_id, result.confidence)); // 存储边界框
        for (const auto& kp : result.key_points) {
            contours.back().push_back(kp); // ✅ 逐个添加点
        }

        DrawDetectedObject(frame, result); // 绘制检测结果
    }


}

// 边界框坐标缩放方法
cv::Rect Inference::GetBoundingBox(const cv::Rect &src) const {
    cv::Rect box = src;
    // 将相对坐标转换为原始图像坐标
    box.x = static_cast<int>(box.x * scale_factor_.x);
    box.y = static_cast<int>(box.y * scale_factor_.y);
    box.width = static_cast<int>(box.width * scale_factor_.x);
    box.height = static_cast<int>(box.height * scale_factor_.y);
    return box;
}

// 关键点坐标缩放方法
std::vector<cv::Point2f> Inference::GetBoundingkeypoints(
    const std::vector<cv::Point2f>& keypoints) const {
    std::vector<cv::Point2f> scaled_keypoints;
    
    for (const auto& kpt : keypoints) {
        scaled_keypoints.emplace_back(
            static_cast<int>(kpt.x * scale_factor_.x),
            static_cast<int>(kpt.y * scale_factor_.y)
        );
    }
    
    return scaled_keypoints;
}

// 绘制检测结果方法
void Inference::DrawDetectedObject(cv::Mat &frame, const Detection &detection) const {
    const cv::Rect &box = detection.box;       // 边界框
    const float &confidence = detection.confidence; // 置信度
    const int &class_id = detection.class_id; // 类别ID
    const std::vector<cv::Point2f> &key_points = detection.key_points; // 关键点坐标
    
    // 生成随机颜色(120-255范围避免深色)
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(120, 255);
    const cv::Scalar color = cv::Scalar(dis(gen), dis(gen), dis(gen));
    
    // 绘制边界框(线宽3像素)
    cv::rectangle(frame, box, color, 3);
    // 绘制关键点(圆形，半径5像素)
    for (const auto &point : key_points) {
        cv::circle(frame, point, 3, color, -1);
    }
    
    // 创建标签文本: 类别名 + 置信度(保留4位)
    std::string classString = classes_[class_id] + " " + std::to_string(confidence).substr(0, 4);
    
    // 计算文本尺寸
    cv::Size textSize = cv::getTextSize(classString, 
                                      cv::FONT_HERSHEY_DUPLEX, 0.75, 2, 0);
    // 创建文本背景框
    cv::Rect textBox(box.x, box.y - textSize.height - 20, 
                    textSize.width + 10, textSize.height + 20);
    
    // 绘制文本背景框(填充)
    cv::rectangle(frame, textBox, color, cv::FILLED);
    
    // 绘制文本(黑色字体，字号0.75，线宽2)
    cv::putText(frame, classString, 
               cv::Point(box.x + 5, box.y - 10), 
               cv::FONT_HERSHEY_DUPLEX, 0.75, cv::Scalar(0, 0, 0), 2);
}
} // namespace yolo