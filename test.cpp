#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

int main() {
	cv::Mat test;
	cv::VideoCapture cap = cv::VideoCapture("http://www.w3schools.com/css/trolltunga.jpg");

	cap >> test;

	cv::Mat gray;

	cv::cvtColor(test, gray, CV_BGR2GRAY);

	cv::imshow("image", gray);
	cv::waitKey(0);
	return 0;
}
