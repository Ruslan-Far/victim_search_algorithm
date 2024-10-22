#ifndef YOLO_INFERENCE_H_
#define YOLO_INFERENCE_H_

#include <string>
#include <vector>

// #include "ros/ros.h"
// #include "sensor_msgs/Image.h"

#include <opencv2/imgproc.hpp>
#include <openvino/openvino.hpp>

namespace yolo {

struct Detection {
	short class_id;
	float confidence;
	cv::Rect box;
};

class Inference {
 public:
	Inference() {}
	Inference(const std::string &model_path, const float &model_confidence_threshold);
	Inference(const std::string &model_path, const cv::Size model_input_shape, const float &model_confidence_threshold);

	std::vector<Detection> RunInference(const cv::Mat &frame);

	// const std::string NODE_NAME = "yolov10_node";
	// const std::string IMG_SUB_TOPIC = "/usb_cam_node/image_raw";
	// const std::string IMG_PUB_TOPIC = "/detected/stereo/left/image_raw";
	// const std::string DET_STATUS_SUB_TOPIC = "/gui_node/det_status";

 private:
	// ros::NodeHandle node;
	// ros::Publisher img_publisher;
	// ros::Subscriber img_subscriber;
	// ros::Subscriber det_status_subscriber;

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
};
} // namespace yolo

#endif // YOLO_INFERENCE_H_
