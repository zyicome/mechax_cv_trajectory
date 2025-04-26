#ifndef YOLO_INFERENCE_H_
#define YOLO_INFERENCE_H_

#include <string>
#include <vector>
#include <opencv2/imgproc.hpp>
#include <openvino/openvino.hpp>

namespace yolo {

struct Detection {
	short class_id;
	float confidence;
    std::vector<cv::Point2f> key_points; // 关键点坐标
	cv::Rect box;
};

class Inference {
 public:
	Inference() {}
	// Constructor to initialize the model with default input shape
	Inference(const std::string &model_path, const float &model_confidence_threshold, const float &model_NMS_threshold);
	// Constructor to initialize the model with specified input shape
	Inference(const std::string &model_path, const cv::Size model_input_shape, const float &model_confidence_threshold, const float &model_NMS_threshold);

	void RunInference(cv::Mat &frame);

    std::vector<std::vector<cv::Point2f>> contours;

 private:
	void InitializeModel(const std::string &model_path);
	void Preprocessing(const cv::Mat &frame);
	void PostProcessing(cv::Mat &frame);
	cv::Rect GetBoundingBox(const cv::Rect &src) const;
    std::vector<cv::Point2f> GetBoundingkeypoints(
        const std::vector<cv::Point2f>& keypoints) const;
	void DrawDetectedObject(cv::Mat &frame, const Detection &detections) const;
    
	cv::Point2f scale_factor_;			// Scaling factor for the input frame
	cv::Size2f model_input_shape_;	// Input shape of the model
	cv::Size model_output_shape_;		// Output shape of the model

	ov::InferRequest inference_request_;  // OpenVINO inference request
	ov::CompiledModel compiled_model_;    // OpenVINO compiled model

	float model_confidence_threshold_;  // Confidence threshold for detections
	float model_NMS_threshold_;         // Non-Maximum Suppression threshold




    
    // buff_names:
	//   0: RR
	//   1: RW
	//   2: BR
	//   3: BW
	std::vector<std::string> classes_ {
        "RR", "RW", "BR", "BW"
	};

};

} // namespace yolo

#endif // YOLO_INFERENCE_H_