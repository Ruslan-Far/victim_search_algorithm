#include "yolov10_node.h"

// #include "inference.h"
// #include "utils.h"

// #include <iostream>
// #include <opencv2/highgui.hpp>


void reset_fields() {
	img_callback_count = 0;
	time_took_sum = 0;
	time_took_count = 0;
	ROS_INFO("reset_fields");
}


// на основе N итераций посчитать, сколько времени в среднем занимает вывод нейросети за одну итерацию
void print_time_took_mean_sum(unsigned int time_took) {
	time_took_sum += time_took;
	time_took_count += 1;
	if time_took_count == 10 { // N == 10
		ROS_INFO("time_took_mean_sum: %f", (float) time_took_sum / time_took_count);
		time_took_sum = 0;
		time_took_count = 0;
	}
}


void run_img_publisher(const cv::Mat& img_rgb, cv_bridge::CvImagePtr& cv_ptr) {
	cv_ptr->image = img_rgb;
	img_publisher.publish(cv_ptr->toImageMsg());
}


// получаем статус режима "human_detection" в gui и переключаем
void det_status_callback(const hum_det::DetectionStatus::ConstPtr& msg) {
	if is_on != msg.is_on {
		is_on = msg.is_on;
		if is_on {
			reset_fields();
		}
	}
}


void img_callback(const sensor_msgs::Image::ConstPtr& msg) {
    // ROS_INFO("I heard: [%s]", msg->data.c_str());
	cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
	cv::Mat img_rgb = cv_ptr->image;
    // cv::Mat hsvImage;
    // cv:cvtColor(image, hsvImage, CV_BGR2HSV);
	if img_callback_count == 0 {
		unsigned int start_time = clock();
		std::vector<yolo::Detection> detections = inference.RunInference(img_rgb);
		unsigned int time_took = clock() - start_time;
		ROS_INFO("time_took: %d", time_took);
		print_time_took_mean_sum(time_took);
		DrawDetectedObject(img_rgb, detections, CLASS_NAMES);
		// отправить для показа в GUI
		run_img_publisher(img_rgb, cv_ptr);
		// просто для показа
		cv::imshow(NODE_NAME, img_rgb);
		cv::waitKey(1)
	}
	img_callback_count += 1;
	if img_callback_count == FREQ {
		img_callback_count = 0
	}
}


int main(const int argc, const char **argv) {
    ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle node;

	// if (argc != 3) {
	// 	std::cerr << "usage: " << argv[0] << " <model_path> <image_path>" << std::endl;
	// 	return 1;
	// }

	// const std::string IMG_PUB_TOPIC = "/detected/stereo/left/image_raw";
	// const std::string IMG_SUB_TOPIC = "/usb_cam_node/image_raw";
	// const std::string DET_STATUS_SUB_TOPIC = "/gui_node/det_status";

	img_publisher = node.advertise<sensor_msgs::Image>(IMG_PUB_TOPIC, 1);
	img_subscriber = node.subscribe(IMG_SUB_TOPIC, 1, img_callback);
	det_status_subscriber = node.subscribe(DET_STATUS_SUB_TOPIC, 1, det_status_callback);
	
	// const std::string model_path = "..."???;
	// const std::string image_path = argv[2];
  	const std::size_t POS = MODEL_PATH.find_last_of("/");
	const std::string METADATA_PATH = model_path.substr(0, pos + 1) + "metadata.yaml";

	CLASS_NAMES = GetClassNameFromMetadata(METADATA_PATH);

	// cv::Mat image = cv::imread(image_path);

	// if (image.empty()) {
	// 	std::cerr << "ERROR: image is empty" << std::endl;
	// 	return 1;
	// }


	yolo::Inference inference(MODEL_PATH, CONFIDENCE_THRESHOLD);






	// std::vector<yolo::Detection> detections = inference.RunInference(image);

	// DrawDetectedObject(image, detections, class_names);
	// cv::imshow("image", image);







	// const char escape_key = 27;

	// while (cv::waitKey(0) != escape_key);
	ros::spin();
	cv::destroyAllWindows();

	return 0;
}
