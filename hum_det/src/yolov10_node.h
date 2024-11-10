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
const std::string IMG_ORIG_PUB_TOPIC = "/orig/stereo/left/image_raw"; // for experiments
const std::string IMG_SUB_TOPIC = "/usb_cam_node/image_raw";
// const std::string IMG_SUB_TOPIC = "/improved/usb_cam_node/image_raw";
// const std::string IMG_SUB_TOPIC = "/stereo/left/image_raw";
const std::string DET_STATUS_SUB_TOPIC = "/gui_node/det_status";

yolo::Inference *inference;

bool is_on = true;

const short FREQ = 1;
// const short FREQ = 25;
// const short FREQ = 5; // работает почему-то медленно (0.18 сек)
short img_callback_count = 0;
double time_took_sum = 0.0;
short time_took_count = 0;
short range_count = 0; // for experiments
const short RANGE_UPPER_LIMIT = 200; // for experiments

ros::Publisher img_publisher;
ros::Publisher img_orig_publisher; // for experiments
ros::Subscriber img_subscriber;
ros::Subscriber det_status_subscriber;

// const std::string MODEL_PATH = "/home/ruslan/kpfu/magistracy/ml_models/usar_engineer3_ep0-20_yolov10s/best_openvino_conv_model/best.xml";
// const std::string MODEL_PATH = "/home/ruslan/kpfu/magistracy/ml_models/yolov10s_openvino_conv_model/yolov10s.xml";
const std::string MODEL_PATH = "/home/ruslan/kpfu/magistracy/ml_models/usar_engineer6_ep0-20_yolov10s/best_openvino_conv_model/best.xml";
// const std::string MODEL_PATH = "/home/ruslan/kpfu/magistracy/ml_models/usar_engineer7_ep0-20_yolov10s/best_openvino_conv_model/best.xml";

// const std::string MODEL_PATH = "/home/ruslan/kpfu/magistracy/ml_models/yolov10n_openvino_conv_model/yolov10n.xml";
// const std::string MODEL_PATH = "/home/ruslan/kpfu/magistracy/ml_models/usar_engineer6_ep0-20_yolov10n/best_openvino_conv_model/best.xml";

// const std::string MODEL_PATH = "/home/ruslan/kpfu/magistracy/ml_models/yolov10m_openvino_conv_model/yolov10m.xml";
// const std::string MODEL_PATH = "/home/ruslan/kpfu/magistracy/ml_models/usar_engineer6_ep0-20_yolov10m/best_openvino_conv_model/best.xml";

// engineer
// const std::string MODEL_PATH = "/home/lirs/ruslan/kpfu/magistracy/ml_models/usar_engineer6_ep0-20_yolov10s/best_openvino_conv_model/best.xml";

std::vector<std::string> class_names;

const float CONFIDENCE_THRESHOLD = 0.5;
const float NMS_THRESHOLD = 0.5;

#endif
