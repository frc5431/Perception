//STANDARD C INCLUDES
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>

//STANDARD C++ INCLUDES
#include <iostream>
#include <memory>

//PERCEPTION INLCUDES
//OLDSRC
//#include "../include/kinect.hpp"
#include "../include/log.hpp"
//OLDSRC
//#include "../include/mjpgserver.hpp"
#include "../include/processing.hpp"

//BOOST INCLUDES
#include <boost/lexical_cast.hpp>

//OPENCV INCLUDES
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

//LIBFREENECT INCLUDES (OLDSRC)
/*
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
*/

//WPILIB INCLUDES
#include <networktables/NetworkTable.h>

#define PERCEPTION_LOG_TAG "PERCEP" //Logger tag name
#define PERCEPTION_TABLE_NAME "perception" //The network table name
#define PERCEPTION_PULL_URL "http://10.54.31.25/mjpg/video.mjpg"

//bool protonect_shutdown = false; // Whether the running application should shut down.


/*void sigint_handler(int s) {
	protonect_shutdown = true;
}*/

//cv::Mat rgbMat;
//cv::Mat defaultMat = cv::imread("/vision/notconnected.png");
/*cv::Mat pullFrame() {
	if(rgbMat.empty()) return defaultMat;
	return rgbMat;
}*/



//Image dimensions
float xSize = 512.0f, ySize = 424.0f;

//Network Tables
std::shared_ptr<NetworkTable> table;

//OLDSRC
/*void onColorFrame(cv::Mat *rgbImage) {
	//std::cout << "NEW FRAME" << std::endl;
	//rgbMat = *rgbImage;
	cv::resize(*rgbImage, *rgbImage, cv::Size(720, 480));
	cv::flip(*rgbImage, *rgbImage, 1);

	cv::imshow("rgb", *rgbImage);
	cv::waitKey(1);
}*/

template<typename T>
void MLOG(T toLog, bool err = false) {
	Logger::Log(PERCEPTION_LOG_TAG, toLog, err);
}

//OLDSRC
/*
void onDepthFrame(kinect::DepthData depthData) {
	//cv::Mat depth, converted, preprocessed;

	cv::Mat depthMat = cv::Mat(depthData.height, depthData.width, CV_32FC1, depthData.depthRaw) / 4500.0f;
		cv::Mat threshed;
		cv::Mat converted;

		depthMat.convertTo(converted, CV_8UC3);
		std::cout << "DEPTH1: " << depthMat.channels() << std::endl;
		std::cout << "DEPTH2: " << converted.channels() << std::endl;
		//std::cout << "DEPTH2: " << converted.depth() << std::endl;

		cv::threshold(depthMat, threshed, 0.0f, 1.0f, CV_THRESH_BINARY);

		cv::imshow("depth", depthMat);
		cv::waitKey(1);


		//std::cout << "DEPTH3: " << threshed.depth() << std::endl;
		//cv::GaussianBlur(threshed, threshed, cv::Size(5, 5), 1);
	//cv::medianBlur(threshed, threshed, 7);

	//processing::kinectDepth2Mat(depth, depthData, 4500.0f);
	//depth.convertTo(converted, CV_8UC3);
	//processing::kinectScalar(depth, fixedDepth);

	//processing::preProcessing(converted, preprocessed);

	//std::vector<processing::Target> targets = std::vector<processing::Target>();

	//processing::processFrame(preprocessed, depthData, targets);

	//MLOG(SW("GOT ") + SW(targets.size()) + SW(" POSSIBLE TARGETS!"));
}*/

void pullLoop() {

	//Camera frame mat and other frames
	cv::Mat camera_frame, threshed;
	
	//Video capture from mjpg stream
	cv::VideoCapture cap = cv::VideoCapture(PERCEPTION_PULL_URL);

	//Set capture properties
	cap.set(CAP_PROP_FPS, PROCESSING_CAMERA_FPS); 

	//Loop forevet
	while(1) {
		//Pull new frame from capture stream (make sure you constantly pull from the buffer)
		cap >> frame;
		
		cv::Mat threshed;

		processing::preProcessing(frame, threshed); 

		std::vector<processing::Target> targets;

		processing::processFrame(threshed, targets);

		unsigned int target_ind = 0;

		std::cout << "POSSIBLE TARGETS FOUND: " << targets.size() << std::endl;

		for(processing::Target target : targets) {
			std::cout << "IND: " << target_ind << "\n	AREA: "
			<< target.area << "\n	WIDTH: " << target.width << "\n	HEIGHT: " << target.height << "\n	PERIM: " << target.perim << "\n	X: " << target.x << "\n	Y: " << target.y << std::endl;
			target_ind++;
		}

		cv::imshow("image", threshed);
		if(cv::waitKey(30) >= 255) break;

    }

    cv::destroyAllWindows();
}


//Main code
int main() {
	pullLoop(); //Start the Mjpg client pullLoop

	//OLDSRC
	//Start the perception logger
	/*Logger::Init();

	MLOG("Starting vision network table");
	table = NetworkTable::GetTable(PERCEPTION_TABLE_NAME);

	//Kinect object initializer
	kinect::Kinect kinect;

	MLOG("Attempting to connect to the Kinect!");
	//Connect to the Kinect
	kinect.loadKinect();

	//Bitwise frame puller example: KINECT_DEPTH | KINECT_COLOR | KINECT_IR
	kinect.setFrames(KINECT_DEPTH);

	MjpgServer server(8081); // Start server on port 8081
	server.setQuality(1); // Set jpeg quality to 1 (0 - 100)
	server.setResolution(640, 320); // Set stream resolution to 1280x720
	server.setFPS(30); // Set target fps to 15
	server.setMaxConnections(3);
	server.setName("Perception Server");
	server.attach(pullFrame);

	//Attach the frame callbacks
	kinect.attachRGB(onColorFrame);
	kinect.attachDepth(onDepthFrame);

	//Start the kinect loop
	kinect.startLoop();
	//server.run(); //Run stream forever (until fatal)

	cv::destroyAllWindows();

	//Free the perception logger
	Logger::Free();*/

	return (0);
}
