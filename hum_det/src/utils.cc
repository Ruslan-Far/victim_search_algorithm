#include "utils.h"

#include <fstream>
#include <yaml-cpp/yaml.h>
#include <cmath>


void DrawDetectedObject(cv::Mat &frame, const std::vector<yolo::Detection> &detections, const std::vector<std::string> &class_names, const float &distance) {	
	for (const auto &detection : detections) {
		const cv::Rect &box = detection.box;
		const float &confidence = round(detection.confidence * 100.0) / 100.0;
		const int &class_id = detection.class_id;
		
		cv::rectangle(frame, box, COLORS[class_id], 3);

		std::string class_string;

		if (class_names.empty())
			class_string = "id[" + std::to_string(class_id) + "] " + std::to_string(confidence).substr(0, 4);
		else
			class_string = class_names[class_id] + " " + std::to_string(confidence).substr(0, 4);
		if (distance != -1) // distance: m -> cm
			class_string = std::to_string((int) (distance * 100)) + "cm " + class_string;

		const cv::Size text_size = cv::getTextSize(class_string, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, 0);

		int text_box_x = box.x - 2;
		int text_box_y = box.y - 27;
		int text_box_width = text_size.width + 10;
		int text_box_height = text_size.height + 15;

		if (text_box_x < 0)
			text_box_x -= text_box_x;
		else if (text_box_x + text_box_width > frame.cols)
			text_box_x += frame.cols - (text_box_x + text_box_width);
		if (text_box_y < 0)
			text_box_y -= text_box_y;
		else if (text_box_y + text_box_height > frame.rows)
			text_box_y += frame.rows - (text_box_y + text_box_height);

		const cv::Rect text_box(text_box_x, text_box_y, text_box_width, text_box_height);

		cv::rectangle(frame, text_box, COLORS[class_id], cv::FILLED);
		cv::putText(frame, class_string, cv::Point(text_box.x + 5, text_box.y + text_box.height - 8), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2, 0);
	}
}


std::vector<std::string> GetClassNameFromMetadata(std::string &metadata_path) {
	std::ifstream check_file(metadata_path);

	if (!check_file.is_open()) {
		std::cerr << "Unable to open file: " << metadata_path << std::endl;
		return {};
	}

	check_file.close();

	YAML::Node metadata = YAML::LoadFile(metadata_path);
	std::vector<std::string> class_names;

	if (!metadata["names"]) {
		std::cerr << "ERROR: 'names' node not found in the YAML file" << std::endl;
		return {};
	}

	for (int i = 0; i < metadata["names"].size(); ++i) {
		std::string class_name = metadata["names"][std::to_string(i)].as<std::string>();
		class_names.push_back(class_name);
	}

	return class_names;
}
