#include "yolov10_node.h"


void reset_fields(bool is_reseted_img_callback_count) {
	if (is_reseted_img_callback_count) {
		img_callback_count = 0;
	}
	time_took_sum = 0.0;
	time_took_count = 0;
	ROS_INFO("reset_fields");
}


// на основе N итераций посчитать, сколько времени в среднем занимает вывод нейросети за одну итерацию
void print_time_took_mean_sum(double time_took) {
	time_took_sum += time_took;
	time_took_count += 1;
	if (time_took_count == 100) { // N == 100
		ROS_INFO("time_took_mean_sum: %f", time_took_sum / time_took_count);
		reset_fields(false);
	}
}


void run_img_publisher(const cv::Mat& img_rgb, cv_bridge::CvImagePtr& cv_ptr) {
	cv_ptr->image = img_rgb;
	img_publisher.publish(cv_ptr->toImageMsg());
}


// получаем статус режима "human_detection" в gui и переключаем
void det_status_callback(const hum_det::DetectionStatus::ConstPtr& msg) {
	if (is_on != msg->is_on) {
		is_on = msg->is_on;
		if (is_on) {
			reset_fields(true);
		}
	}
}


void img_callback(const sensor_msgs::Image::ConstPtr& msg) {
	if (!is_on) {
		return;
	}

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	cv::Mat img_rgb = cv_ptr->image;
	if (img_callback_count == 0) {
		auto start_time = std::chrono::high_resolution_clock::now();
		std::vector<yolo::Detection> detections = (*inference).RunInference(img_rgb);
		auto end_time = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> time_took = end_time - start_time;
		ROS_INFO("time_took: %f", time_took.count());
		print_time_took_mean_sum(time_took.count());
		DrawDetectedObject(img_rgb, detections, class_names);
		// отправить для показа в GUI
		run_img_publisher(img_rgb, cv_ptr);
		// просто для показа
		cv::imshow(NODE_NAME, img_rgb);
		cv::waitKey(1);
	}
	img_callback_count += 1;
	if (img_callback_count == FREQ) {
		img_callback_count = 0;
	}
}


int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle node;

	img_publisher = node.advertise<sensor_msgs::Image>(IMG_PUB_TOPIC, 1);
	img_subscriber = node.subscribe(IMG_SUB_TOPIC, 1, img_callback);
	det_status_subscriber = node.subscribe(DET_STATUS_SUB_TOPIC, 1, det_status_callback);
	
  	const std::size_t POS = MODEL_PATH.find_last_of("/");
	std::string metadata_path = MODEL_PATH.substr(0, POS + 1) + "metadata.yaml";
	class_names = GetClassNameFromMetadata(metadata_path);

	inference = new yolo::Inference(MODEL_PATH, CONFIDENCE_THRESHOLD);

	ros::spin();
	cv::destroyAllWindows();

	return 0;
}
