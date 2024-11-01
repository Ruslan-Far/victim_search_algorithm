#ifndef YOLOV10_NODE_H
# define YOLOV10_NODE_H

# include <ros/ros.h>
# include <sensor_msgs/Image.h>
# include <sensor_msgs/image_encodings.h>
# include "hum_det/DetectionStatus.h"
# include "inference.h"
# include "utils.h"
# include <cv_bridge/cv_bridge.h>
# include <chrono>
# include <iostream>
# include <opencv2/highgui.hpp>

const std::string NODE_NAME = "yolov10_node";
const std::string IMG_PUB_TOPIC = "/detected/stereo/left/image_raw";
// const std::string IMG_SUB_TOPIC = "/usb_cam_node/image_raw";
const std::string IMG_SUB_TOPIC = "/stereo/left/image_raw";
const std::string DET_STATUS_SUB_TOPIC = "/gui_node/det_status";

yolo::Inference *inference;

bool is_on = true;

const short FREQ = 1;
short img_callback_count = 0;
double time_took_sum = 0.0;
short time_took_count = 0;

ros::Publisher img_publisher;
ros::Subscriber img_subscriber;
ros::Subscriber det_status_subscriber;

// const std::string MODEL_PATH = "/home/ruslan/kpfu/magistracy/ml_models/usar_engineer3_ep0-20_yolov10s/best_openvino_conv_model/best.xml";
// const std::string MODEL_PATH = "/home/ruslan/kpfu/magistracy/ml_models/yolov10s_openvino_conv_model/yolov10s.xml";
// const std::string MODEL_PATH = "/home/ruslan/kpfu/magistracy/ml_models/usar_engineer6_ep0-20_yolov10s/best_openvino_conv_model/best.xml";

// engineer
const std::string MODEL_PATH = "/home/lirs/ruslan/kpfu/magistracy/ml_models/usar_engineer6_ep0-20_yolov10s/best_openvino_conv_model/best.xml";

std::vector<std::string> class_names;

const float CONFIDENCE_THRESHOLD = 0.5;

#endif
