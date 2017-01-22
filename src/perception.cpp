//STANDARD C INCLUDES
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>

//STANDARD C++ INCLUDES
#include <iostream>
#include <memory>

//PERCEPTION INLCUDES
#include "../include/kinect.hpp"
#include "../include/log.hpp"
#include "../include/mjpgserver.hpp"

//BOOST INCLUDES
#include <boost/lexical_cast.hpp>

//OPENCV INCLUDES
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

//LIBFREENECT INCLUDES
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

//WPILIB INCLUDES
#include <networktables/NetworkTable.h>

#define PERCEPTION_LOG_TAG "PERCEP" //Logger tag name
#define PERCEPTION_TABLE_NAME "perception" //The network table name

//bool protonect_shutdown = false; // Whether the running application should shut down.


/*void sigint_handler(int s) {
	protonect_shutdown = true;
}*/

cv::Mat rgbMat;
cv::Mat defaultMat = cv::imread("/vision/notconnected.png");

cv::Mat pullFrame() {
	if(rgbMat.empty()) return defaultMat;
	return rgbMat;
}



//Image dimensions
float xSize = 512.0f, ySize = 424.0f;

//Network Tables
std::shared_ptr<NetworkTable> table;


void onRGBFrame(cv::Mat *rgbImage) {
	//std::cout << "NEW FRAME" << std::endl;
	//rgbMat = *rgbImage;
	/*cv::resize(*rgbImage, *rgbImage, cv::Size(720, 480));
	cv::flip(*rgbImage, *rgbImage, 1);

	cv::imshow("rgb", *rgbImage);
	cv::waitKey(1);*/
}

template<typename T>
void MLOG(T toLog, bool err = false) {
	Logger::Log(PERCEPTION_LOG_TAG, toLog, err);
}

void onDepthFrame(kinect::DepthData depthData) {
	cv::Mat depthMat = cv::Mat(depthData.height, depthData.width, CV_32FC1, depthData.depthRaw) /4500.0f, threshed;
	cv::Mat converted;

	depthMat.convertTo(converted, CV_8UC3);
	cv::threshold(converted, threshed, 0, 100, CV_THRESH_BINARY_INV);
	//cv::GaussianBlur(threshed, threshed, cv::Size(5, 5), 1);
	cv::medianBlur(threshed, threshed, 7);

	int erosion_size = 1;

	cv::Mat eroder = cv::getStructuringElement(
				cv::MORPH_ERODE, cv::Size( 2*erosion_size + 1,
				2*erosion_size+1 ),cv::Point( erosion_size, erosion_size ));

	cv::erode(threshed, threshed, eroder);


	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(threshed, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

	cv::Mat drawing = cv::Mat::zeros(threshed.size(), CV_8UC3);
	int ind = 0;

	for(std::vector<cv::Point> contour : contours) {

		float area = cv::contourArea(contour);

		if(area > 20 && area < 500) {
			//std::cout << "NEW FRAME" << std::endl;
			//rgbMat = *rgbImage;
			/*cv::resize(*rgbImage, *rgbImage, cv::Size(720, 480));
			cv::flip(*rgbImage, *rgbImage, 1);

			cv::imshow("rgb", *rgbImage);
			cv::waitKey(1);*/
			std::vector<cv::Point> approx;
			cv::approxPolyDP(contour, approx, 5, true);

			float sides = approx.size();

			if(sides > 2 && sides < 4) {

				float length = cv::arcLength(contour, true);

				if(length > 30 && length < 200) {

					cv::Rect bound = cv::boundingRect(contour);

					float targetX = static_cast<float>(bound.height) * 2.0f;

					if(static_cast<float>(bound.width) >= targetX && bound.height < 50) {
						cv::Moments mu = cv::moments(contour, false);

						float x = (mu.m10 / mu.m00);
						float y = ySize - (mu.m01 / mu.m00) ;

						if(y > 250 && y < ySize - 10) {
							std::cout << "SIDES: " << sides;
							std::cout << " AREA: " << area;
							std::cout << " LENGTH: " << length;
							std::cout << " WIDTH: " << bound.width;
							std::cout << " HEIGHT: " << bound.height;
							std::cout << " X: " << x  << " Y: " << y << std::endl;

							//table->PutNumber()

							cv::Scalar color = cv::Scalar(255, 0, 0);
							cv::drawContours(drawing, contours, ind, color, 2, 8, hierarchy, 0, cv::Point());
						}
					}
				}
			}
		}

		ind++;
	}

	//cv::circle(depthMat, cv::Point(200, 200), 5, cv::Scalar(1.0f, 1.0f, 1.0f), 2);

	//std::cout << "DEPTH: " << depthData.getDepth(&depthMat, 200, 200) << std::endl;
	cv::imshow("ir", threshed);
	cv::imshow("contours", drawing);
	cv::waitKey(1);

	//std::cout << "Depth: " << depthData.getDepth(250, 250) << std::endl;
}

//Main code
int main() {

	//Start the perception logger
	Logger::Init();

	MLOG("Starting vision network table");
	table = NetworkTable::GetTable(PERCEPTION_TABLE_NAME);

	//Kinect object initializer
	kinect::Kinect kinect;

	MLOG("Attempting to connect to the Kinect!");
	//Connect to the Kinect
	kinect.loadKinect();

	//Bitwise frame puller example: KINECT_DEPTH | KINECT_COLOR | KINECT_IR
	kinect.setFrames(KINECT_DEPTH);

    /*MjpgServer server(8081); // Start server on pot 8081
    server.setQuality(1); // Set jpeg quality to 1 (0 - 100)
    server.setRes	Logger::
	cv::threshold();olution(640, 320); // Set stream resolution to 1280x720
    server.setFPS(30); // Set target fps to 15
    server.setMaxConnections(3);
    server.setName("Perception Server");
	server.attach(pullFrame);*/

	//Attach the frame callbacks
	kinect.attachRGB(onRGBFrame);
	kinect.attachIR(onDepthFrame);

	//Start the kinect loop
	kinect.startLoop();
    //server.run(); //Run stream forever (until fatal)

	cv::destroyAllWindows();

	//Free the perception logger
	Logger::Free();

	return (0);
}
