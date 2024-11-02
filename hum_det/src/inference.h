#ifndef YOLO_INFERENCE_H_
# define YOLO_INFERENCE_H_

# include <string>
# include <vector>

# include <opencv2/imgproc.hpp>
# include <openvino/openvino.hpp>

namespace yolo {

struct Detection {
	cv::Rect box;
	float confidence;
	short class_id;
};

class Inference {
 public:
	Inference() {}
	Inference(const std::string &model_path, const float &model_confidence_threshold, const float &model_nms_threshold);
	Inference(const std::string &model_path, const cv::Size model_input_shape, const float &model_confidence_threshold);

	std::vector<Detection> RunInference(const cv::Mat &frame);

 private:

	void InitialModel(const std::string &model_path);
	void Preprocessing(const cv::Mat &frame);
	void PostProcessing();
	cv::Rect GetBoundingBox(const cv::Rect &src) const;

	cv::Point2f scale_factor_;
	cv::Size2f model_input_shape_;
	cv::Size model_output_shape_;

	ov::InferRequest inference_request_;
	ov::CompiledModel compiled_model_;

	std::vector<Detection> detections_;

	float model_confidence_threshold_;
	float model_nms_threshold_;
};
}

#endif
