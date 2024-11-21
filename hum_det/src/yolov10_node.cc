#include "yolov10_node.h"


void reset_fields(bool will_be_reseted_camera_img_callback_count) {
	if (will_be_reseted_camera_img_callback_count) {
		camera_img_callback_count = 0;
		range_count = 0; // for experiments
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


void run_det_img_publisher(const cv::Mat& img, cv_bridge::CvImagePtr& cv_ptr) {
	cv_ptr->image = img;
	det_img_publisher.publish(cv_ptr->toImageMsg());
}


// for experiments
void run_orig_img_publisher(const cv::Mat& img, cv_bridge::CvImagePtr& cv_ptr) {
	cv_ptr->image = img;
	orig_img_publisher.publish(cv_ptr->toImageMsg());
}


// получаем статус режима "human_detection" в gui и переключаем
void gui_det_status_callback(const hum_det::DetectionStatus::ConstPtr& msg) {
	if (is_on != msg->is_on) {
		is_on = msg->is_on;
		if (is_on) {
			reset_fields(true);
		}
	}
}


void camera_img_callback(const sensor_msgs::Image::ConstPtr& msg) {
	if (!is_on) {
		return;
	}

	if (camera_img_callback_count == 0) {
		// cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // laptop
    	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8); // engineer
		cv::Mat img_rgb = cv_ptr->image;
		cv::Mat img_bgr; // engineer
		// for experiments (engineer)
		cv::cvtColor(img_rgb, img_bgr, CV_RGB2BGR);
		run_orig_img_publisher(img_bgr, cv_ptr); // не влияет на скорость
		//
		auto start_time = std::chrono::high_resolution_clock::now();
		std::vector<yolo::Detection> detections = (*inference).RunInference(img_rgb);
		auto end_time = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> time_took = end_time - start_time;
		ROS_INFO("time_took: %f", time_took.count());
		print_time_took_mean_sum(time_took.count());
		DrawDetectedObject(img_rgb, detections, class_names);
		// отправить для показа в GUI (engineer)
		cv::cvtColor(img_rgb, img_bgr, CV_RGB2BGR);
		run_det_img_publisher(img_bgr, cv_ptr);
		//
		// просто для показа
		// cv::imshow(NODE_NAME, img_rgb);
		// cv::waitKey(1);
		// for experiments
		if (range_count == RANGE_UPPER_LIMIT - 1)
			is_on = false;
		else
			range_count++;
		//
	}
	camera_img_callback_count += 1;
	if (camera_img_callback_count == FREQ) {
		camera_img_callback_count = 0;
	}
}
// for experiments
// rosbag record /yolov10_node/stereo/left/det_image
// rosbag record /yolov10_node/stereo/left/orig_image


int main(int argc, char **argv) {
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle node;

	det_img_publisher = node.advertise<sensor_msgs::Image>(DET_IMG_TOPIC, 1); // работает с 0 и 1000 как обычно и время работы не меняется
	orig_img_publisher = node.advertise<sensor_msgs::Image>(ORIG_IMG_TOPIC, 1);
	camera_img_subscriber = node.subscribe(CAMERA_IMG_TOPIC, 1, camera_img_callback); // работает с 0 и 1000 очень странно, но время работы не меняется
	// Это работает в замедленном действии: быстро поступающие изображения все сохраняются в буфер и обрабатываются последовательно
	gui_det_status_subscriber = node.subscribe(GUI_DET_STATUS_TOPIC, 1, gui_det_status_callback);
	
	const std::size_t POS = MODEL_PATH.find_last_of("/");
	std::string metadata_path = MODEL_PATH.substr(0, POS + 1) + "metadata.yaml";
	class_names = GetClassNameFromMetadata(metadata_path);

	inference = new yolo::Inference(MODEL_PATH, CONFIDENCE_THRESHOLD, NMS_THRESHOLD);

	ros::spin();
	cv::destroyAllWindows();

	return 0;
}









// // необходимо для одного вывода прочитанного изображения
// void process_img(cv::Mat& img_rgb) {
// 	auto start_time = std::chrono::high_resolution_clock::now();
// 	std::vector<yolo::Detection> detections = (*inference).RunInference(img_rgb);
// 	auto end_time = std::chrono::high_resolution_clock::now();
// 	std::chrono::duration<double> time_took = end_time - start_time;
// 	ROS_INFO("time_took: %f", time_took.count());
// 	DrawDetectedObject(img_rgb, detections, class_names);
// 	// просто для показа
// 	cv::imshow(NODE_NAME, img_rgb);
// 	cv::waitKey(0);
// }


// // просто прочесть одно изображение и вывести на экран уже обработанное изображение
// int main(int argc, char **argv) {
// 	const std::size_t POS = MODEL_PATH.find_last_of("/");
// 	std::string metadata_path = MODEL_PATH.substr(0, POS + 1) + "metadata.yaml";
// 	class_names = GetClassNameFromMetadata(metadata_path);
	
// 	inference = new yolo::Inference(MODEL_PATH, CONFIDENCE_THRESHOLD, NMS_THRESHOLD);

// 	std::string img_path = argv[1];
// 	cv::Mat img_rgb = cv::imread(img_path);
// 	// cv::cvtColor(img_rgb, img_rgb, cv::COLOR_RGB2BGR); // эксперементировал
// 	// cv::resize(img_rgb, img_rgb, cv::Size(640, 640), 0, 0, cv::INTER_AREA); // эксперементировал

// 	// cv::imshow(NODE_NAME, img_rgb); // engineer (иначе не будет работать)
// 	// cv::waitKey(0); // engineer (иначе не будет работать)

// 	process_img(img_rgb);

// 	cv::destroyAllWindows();

// 	return 0;
// }
