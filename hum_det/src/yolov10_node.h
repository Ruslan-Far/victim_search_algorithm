#ifndef YOLOV10_NODE_H
# define YOLOV10_NODE_H

# include <ros/ros.h>
# include <sensor_msgs/Image.h>
# include <sensor_msgs/image_encodings.h>
# include <stereo_msgs/DisparityImage.h>
# include "hum_det/Det.h"
# include "hum_det/DetArray.h"
# include "hum_det/ModeSwitch.h"
# include "inference.h"
# include "utils.h"
# include <cv_bridge/cv_bridge.h>
# include <chrono>
# include <iostream>
# include <opencv2/highgui.hpp>
# include <unistd.h>

const std::string NODE_NAME = "yolov10_node";

const std::string DET_IMG_TOPIC = "/" + NODE_NAME + "/stereo/left/det_image";
// const std::string CAMERA_IMG_TOPIC = "/usb_cam_node/image_raw";
// const std::string CAMERA_IMG_TOPIC = "/narrow_stereo_textured/left/image_rect_color";
const std::string CAMERA_IMG_TOPIC = "/wide_stereo/left/image_raw";
// const std::string CAMERA_IMG_TOPIC = "/improved/usb_cam_node/image_raw";
// const std::string CAMERA_IMG_TOPIC = "/stereo/left/image_raw";
const std::string CAMERA_DISP_IMG_TOPIC = "/wide_stereo/disparity";
const std::string DET_ARRAY_TOPIC = "/det_array";
const std::string GOAL_DET_TOPIC = "/goal_det";

const std::string DET_MODE_SWITCH_SRV = "det_mode_switch";

extern std::unique_ptr<yolo::Inference> inference;

bool is_on = false;

const short FREQ = 1;
short camera_img_callback_count = 0;
double time_took_sum = 0.0;
short time_took_count = 0;

ros::Publisher det_img_publisher;
ros::Publisher det_array_pub;
ros::Subscriber camera_img_subscriber;
ros::Subscriber goal_det_sub;

ros::ServiceServer det_mode_switch_server;

// laptop
const std::string MODEL_PATH = "/home/ruslan/kpfu/magistracy/ml_models/usar_engineer6_ep0-20_yolov10s/best_openvino_conv_model/best.xml";

// engineer
// const std::string MODEL_PATH = "/home/lirs/ruslan/kpfu/magistracy/ml_models/usar_engineer6_ep0-20_yolov10s/best_openvino_conv_model/best.xml";

std::vector<std::string> class_names;

const float CONFIDENCE_THRESHOLD = 0.5;
const float NMS_THRESHOLD = 0.5;

#endif
