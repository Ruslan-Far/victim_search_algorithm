#ifndef UTILS_H_
# define UTILS_H_

# include "inference.h"

const cv::Scalar COLORS[4] = {cv::Scalar(0, 0, 255), cv::Scalar(203, 192, 255), cv::Scalar(0, 102, 255), cv::Scalar(0, 255, 255)};

void DrawDetectedObject(cv::Mat &frame, const std::vector<yolo::Detection> &detections, const std::vector<std::string> &class_names, const float &distance);
std::vector<std::string> GetClassNameFromMetadata(std::string &metadata_path);

#endif
