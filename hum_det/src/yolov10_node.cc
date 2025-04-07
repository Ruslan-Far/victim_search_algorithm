#include "yolov10_node.h"

std::unique_ptr<yolo::Inference> inference = nullptr;


void reset_fields(bool will_be_reseted_camera_img_callback_count) {
	if (will_be_reseted_camera_img_callback_count) {
		camera_img_callback_count = 0;
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


void run_det_img_pub(const cv::Mat &img, cv_bridge::CvImagePtr &cv_ptr) {
	cv_ptr->image = img;
	det_img_pub.publish(cv_ptr->toImageMsg());
}


void run_det_array_pub(const std::vector<yolo::Detection> &detections, const sensor_msgs::Image::ConstPtr &msg_img,
																const stereo_msgs::DisparityImage::ConstPtr msg_disp_img) {
	hum_det::DetArray msg;
	hum_det::Det msg_det;

	for (const auto &detection : detections) {
		msg_det.bbox.x = detection.box.x;
		msg_det.bbox.y = detection.box.y;
		msg_det.bbox.w = detection.box.width;
		msg_det.bbox.h = detection.box.height;
		msg_det.confidence = detection.confidence;
		msg_det.class_id = detection.class_id;
		msg.dets.push_back(msg_det);
	}
	msg.img = *msg_img;
	if (msg_disp_img) {
		ROS_INFO("disparity message received!");
		msg.disp_img = *msg_disp_img;
	}
	else {
		ROS_WARN("failed to get disparity message in 1 second");
		stereo_msgs::DisparityImage disp_img;
		disp_img.f = -1.0;
		msg.disp_img = disp_img;
	}
	det_array_pub.publish(msg);
}


void camera_img_callback(const sensor_msgs::Image::ConstPtr &msg) {
	if (!is_on) {
		return;
	}

	if (camera_img_callback_count == 0) {
		// получаем диспаритетное изображение (ждем 1 секунду) {
		boost::shared_ptr<stereo_msgs::DisparityImage const> msg_disp_img;
		msg_disp_img = ros::topic::waitForMessage<stereo_msgs::DisparityImage>(CAMERA_DISP_IMG_TOPIC, ros::Duration(1.0));
		// }
		cv_bridge::CvImagePtr cv_ptr;
		if (IS_LAPTOP)
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // laptop
		else
    		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8); // engineer
		cv::Mat img_rgb = cv_ptr->image;
		auto start_time = std::chrono::high_resolution_clock::now();
		std::vector<yolo::Detection> detections = (*inference).RunInference(img_rgb);
		auto end_time = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> time_took = end_time - start_time;
		ROS_INFO("time_took: %f", time_took.count());
		print_time_took_mean_sum(time_took.count());
		run_det_array_pub(detections, msg, msg_disp_img);
		DrawDetectedObject(img_rgb, detections, class_names, -1);
		if (IS_LAPTOP) {
			// отправить для показа в GUI (laptop)
			run_det_img_pub(img_rgb, cv_ptr);
		}
		else {
			// отправить для показа в GUI (engineer)
			cv::Mat img_bgr;
			cv::cvtColor(img_rgb, img_bgr, CV_RGB2BGR);
			run_det_img_pub(img_bgr, cv_ptr);
		}
	}
	camera_img_callback_count += 1;
	if (camera_img_callback_count == FREQ) {
		camera_img_callback_count = 0;
	}
}


void goal_det_callback(const hum_det::DetArray::ConstPtr &msg) {
	cv_bridge::CvImagePtr cv_ptr;
	if (IS_LAPTOP)
		cv_ptr = cv_bridge::toCvCopy(msg->img, sensor_msgs::image_encodings::BGR8); // laptop
	else
		cv_ptr = cv_bridge::toCvCopy(msg->img, sensor_msgs::image_encodings::RGB8); // engineer
	cv::Mat img_rgb = cv_ptr->image;
	std::vector<yolo::Detection> detections;

	for (const auto &msg_det : msg->dets) {
		yolo::Detection result;
		result.box.x = msg_det.bbox.x;
		result.box.y = msg_det.bbox.y;
		result.box.width = msg_det.bbox.w;
		result.box.height = msg_det.bbox.h;
		result.confidence = msg_det.confidence;
		result.class_id = msg_det.class_id;
		detections.push_back(result);
	}
	ROS_INFO("msg->distance = %f", msg->distance);
	DrawDetectedObject(img_rgb, detections, class_names, msg->distance);
	if (IS_LAPTOP) {
		// отправить для показа в GUI (laptop)
		run_det_img_pub(img_rgb, cv_ptr);
	}
	else {
		// отправить для показа в GUI (engineer)
		cv::Mat img_bgr;
		cv::cvtColor(img_rgb, img_bgr, CV_RGB2BGR);
		run_det_img_pub(img_bgr, cv_ptr);
	}
	ROS_INFO("before 3 seconds");
	sleep(3);
	ROS_INFO("after 3 seconds");
}


// переключаем режим Human Detection во вкл/выкл состояние
bool handle_det_mode_switch(hum_det::ModeSwitch::Request &req, hum_det::ModeSwitch::Response &res) {
	is_on = req.is_on;
	if (is_on) {
		reset_fields(is_on);
	}
	res.code = 0; // операция прошла успешно
	return true;
}


int main(int argc, char **argv) {
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle node;

	det_img_pub = node.advertise<sensor_msgs::Image>(DET_IMG_TOPIC, 1);
	det_array_pub = node.advertise<hum_det::DetArray>(DET_ARRAY_TOPIC, 1);
	camera_img_sub = node.subscribe(CAMERA_IMG_TOPIC, 1, camera_img_callback);
	goal_det_sub = node.subscribe(GOAL_DET_TOPIC, 1, goal_det_callback);
	
	det_mode_switch_server = node.advertiseService(DET_MODE_SWITCH_SRV, handle_det_mode_switch);
	
	const std::size_t POS = MODEL_PATH.find_last_of("/");
	std::string metadata_path = MODEL_PATH.substr(0, POS + 1) + "metadata.yaml";
	class_names = GetClassNameFromMetadata(metadata_path);

	inference = std::make_unique<yolo::Inference>(MODEL_PATH, CONFIDENCE_THRESHOLD, NMS_THRESHOLD);

	ros::spin();
	return 0;
}









// // необходимо для одного вывода прочитанного изображения
// void process_img(cv::Mat &img_rgb) {
// 	auto start_time = std::chrono::high_resolution_clock::now();
// 	std::vector<yolo::Detection> detections = (*inference).RunInference(img_rgb);
// 	auto end_time = std::chrono::high_resolution_clock::now();
// 	std::chrono::duration<double> time_took = end_time - start_time;
// 	ROS_INFO("time_took: %f", time_took.count());
// 	DrawDetectedObject(img_rgb, detections, class_names, -1);
// 	// просто для показа
// 	cv::imshow(NODE_NAME, img_rgb);
// 	cv::waitKey(0);
// }


// // просто прочесть одно изображение и вывести на экран уже обработанное изображение
// int main(int argc, char **argv) {
// 	const std::size_t POS = MODEL_PATH.find_last_of("/");
// 	std::string metadata_path = MODEL_PATH.substr(0, POS + 1) + "metadata.yaml";
// 	class_names = GetClassNameFromMetadata(metadata_path);
	
// 	inference = std::make_unique<yolo::Inference>(MODEL_PATH, CONFIDENCE_THRESHOLD, NMS_THRESHOLD);

// 	// задать путь
// 	// std::string img_path = argv[1];
// 	std::string img_path = "/home/ruslan/kpfu/magistracy/graduate_work/paper/journal/computer_optics/images/frame0043.jpg";
// 	// std::string img_path = "/home/ruslan/kpfu/magistracy/graduate_work/paper/test_1409/scenario_1/range_1_50-200/light/orig/s1r1_50-200_light_orig/frame0027.jpg";
// 	// std::string img_path = "/home/ruslan/kpfu/magistracy/graduate_work/paper/test_1409/scenario_1/range_1_50-200/light/orig/s1r1_50-200_light_orig/frame0040.jpg";
// 	// std::string img_path = "/home/ruslan/kpfu/magistracy/graduate_work/paper/test_1409/scenario_1/range_1_50-200/light/orig/s1r1_50-200_light_orig/frame0043.jpg";
// 	//

// 	cv::Mat img_rgb = cv::imread(img_path);
// 	// cv::cvtColor(img_rgb, img_rgb, cv::COLOR_RGB2BGR); // эксперементировал
// 	// cv::resize(img_rgb, img_rgb, cv::Size(640, 640), 0, 0, cv::INTER_AREA); // экспериментировал

// 	// cv::imshow(NODE_NAME, img_rgb); // engineer (иначе не будет работать)
// 	// cv::waitKey(0); // engineer (иначе не будет работать)

// 	process_img(img_rgb);

// 	// cv::resize(img_rgb, img_rgb, cv::Size(640, 640), 0, 0, cv::INTER_AREA);

// 	// cv::imwrite("/home/ruslan/det_frame0043.jpg", img_rgb);

// 	cv::destroyAllWindows();

// 	return 0;
// }
