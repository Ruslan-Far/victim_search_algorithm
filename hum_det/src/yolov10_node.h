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

const bool IS_LAPTOP = true; // если запуск алгоритма будет производиться на ноутбуке. Для робота Инженер необходимо поставить false

std::string get_camera_img_topic() {
    if (IS_LAPTOP) {
		// return "/narrow_stereo_textured/left/image_rect_color";
        // return "/wide_stereo/left/image_raw";
		return "/usb_cam_node/image_raw";
	}
    else
        return "/stereo/left/image_raw";
}

std::string get_camera_disp_img_topic() {
    if (IS_LAPTOP)
        return "/wide_stereo/disparity";
    else
        return "/stereo/disparity";
}

std::string get_model_path() {
    if (IS_LAPTOP)
        return "/home/ruslan/kpfu/magistracy/ml_models/usar_engineer6_ep0-20_yolov10s/best_openvino_conv_model/best.xml";
    else
        return "/home/lirs/ruslan/kpfu/magistracy/ml_models/usar_engineer6_ep0-20_yolov10s/best_openvino_conv_model/best.xml";
}

const std::string NODE_NAME = "yolov10_node";

const std::string DET_IMG_TOPIC = "/" + NODE_NAME + "/stereo/left/det_image";
const std::string CAMERA_IMG_TOPIC = get_camera_img_topic();
const std::string CAMERA_DISP_IMG_TOPIC = get_camera_disp_img_topic();
const std::string DET_ARRAY_TOPIC = "/det_array";
const std::string GOAL_DET_TOPIC = "/goal_det";

const std::string DET_MODE_SWITCH_SRV = "/det_mode_switch";

const short FREQ = 1;
const std::string MODEL_PATH = get_model_path();
const float CONFIDENCE_THRESHOLD = 0.5;
const float NMS_THRESHOLD = 0.5;

ros::Publisher det_img_pub;
ros::Publisher det_array_pub;
ros::Subscriber camera_img_sub;
ros::Subscriber goal_det_sub;

ros::ServiceServer det_mode_switch_server;

extern std::unique_ptr<yolo::Inference> inference;
bool is_on = false;
short camera_img_callback_count = 0;
double time_took_sum = 0.0;
short time_took_count = 0;
std::vector<std::string> class_names;

#endif
