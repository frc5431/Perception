/*------------------------------------------------------------------------------||
|                                                                                |
| Copyright (C) 2017 by Titan Robotics                                           |
| License Date: 01/23/2017                                                       |
| Modifiers: none                                                                |
|                                                                                |
| Permission is hereby granted, free of charge, to any person obtaining a copy   |
| of this software and associated documentation files (the "Software"), to deal  |
| in the Software without restriction, including without limitation the rights   |
| to use, copy, modify, merge, publish, distribute, sublicense, and/or sell      |
| copies of the Software, and to permit persons to whom the Software is          |
| furnished to do so, subject to the following conditions:                       |
|                                                                                |
| The above copyright notice and this permission notice shall be included in all |
| copies or substantial portions of the Software.                                |
|                                                                                |
| THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR     |
| IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,       |
| FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE    |
| AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER         |
| LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,  |
| OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE  |
| SOFTWARE.                                                                      |
|                                                                                |
||------------------------------------------------------------------------------*/

/**
 *
 * @file perception.cpp
 * @date 01/23/2017
 * @brief Source that includes the main functions for perception
 *
 * @details This source compiles all the processing for perception and it's libraries
 * the main function should only ever call abstracted functions to start perception for
 * quick portability and it's easier to read.
 */

#include "../include/perception.hpp"
#include "../include/gear.hpp"
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

cv::VideoCapture getCapture() {
	while(1) {
		try {
			cv::VideoCapture cap = cv::VideoCapture(PERCEPTION_PULL_URL);

			if(cap.isOpened()) return cap;
		} catch(const cv::Exception &err) {
			std::cout << "Failed initializing capture!" << std::endl;
		} catch(const std::exception &err) {
			std::cout << "Failed initializing capture!" << std::endl;
		}
	}
}

void pullLoop() {
	std::cout << "STARTING" << std::endl;

	boost::thread(Table::init);	

	//Camera frame mat and other frames
	cv::Mat camera_frame, threshed, contoured;
	
	//Video capture from mjpg stream
	cv::VideoCapture axis_camera = getCapture();

	//Set capture properties
	axis_camera.set(CV_CAP_PROP_FPS, PROCESSING_CAMERA_FPS);

	unsigned long empty_frame_count = 0;

	//Loop forevet
	while(1) {
		//table->GetNumber("OK", 0.0);

		//Pull new frame from capture stream (make sure you constantly pull from the buffer)
		axis_camera.read(camera_frame);

		bool passed = true;

		if(camera_frame.empty()) {
			empty_frame_count++;
			std::cout << "empty frame" << std::endl;
			passed = false;
		}

		if(empty_frame_count > PERCEPTION_MAX_FAIL_COUNT) {
			axis_camera = getCapture();
			
			empty_frame_count = 0;
			
			continue;
		}

		if(!passed) {
			cv::waitKey(1);
			continue;
		}

		Logger::LogWindow("Original", camera_frame);

		cv::Mat threshed;

		processing::preProcessing(camera_frame, threshed);

		std::vector<processing::Target> targets;

		processing::processFrame(threshed, targets, contoured);

		unsigned int target_ind = 0;

		//std::cout << "POSSIBLE TARGETS FOUND: " << targets.size() << std::endl;

		/*for(processing::Target target : targets) {
			std::cout << "IND: " << target_ind << "\n	AREA: "
			<< target.area << "\n	WIDTH: " << target.width 
			<< "\n	HEIGHT: " << target.height << "\n	PERIM: " 
			<< target.perim << "\n	X: " << target.x << "\n	Y: " 
			<< target.y << "\n	SIDES: " << target.sides << std::endl;
			target_ind++;
		}*/

		processing::Target final_target;

		processing::processTargets(targets, final_target);

		Table::updateTarget(final_target);

		Logger::LogWindow("Threshed", threshed);
		Logger::LogWindow("Contours", contoured);

		cv::waitKey(20);

		//if(cv::waitKey(30) >= 255) break;

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
