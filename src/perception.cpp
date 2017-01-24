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
#include "../include/processing.hpp"

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


void onColorFrame(cv::Mat *rgbImage) {
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
	cv::Mat depth, fixedDepth, preprocessed;

	processing::kinectDepth2Mat(depth, depthData, 4500.0f);
	processing::kinectScalar(depth, fixedDepth);

	processing::preProcessing(fixedDepth, preprocessed);

	std::vector<processing::Target> targets = std::vector<processing::Target>();

	processing::processFrame(preprocessed, depthData, targets);

	MLOG(SW("GOT ") + SW(targets.size()) + SW(" POSSIBLE TARGETS!"));
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
	kinect.attachRGB(onColorFrame);
	kinect.attachDepth(onDepthFrame);

	//Start the kinect loop
	kinect.startLoop();
    //server.run(); //Run stream forever (until fatal)

	cv::destroyAllWindows();

	//Free the perception logger
	Logger::Free();

	return (0);
}
