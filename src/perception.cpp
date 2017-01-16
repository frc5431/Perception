//STANDARD C INCLUDES
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>

//STANDARD C++ INCLUDES
#include <iostream>

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

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <signal.h>

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

int main() {
	Logger::Init();



	kinect::Kinect kinect;
	kinect.loadKinect();

	kinect.setFrames(KINECT_COLOR | KINECT_IR | KINECT_DEPTH);

	/*while(1) {
		cv::Mat color, ir, depth;

		if(!kinect.pullAll()) {
			continue;
		}

		color = kinect.getRGBMat();
		ir = kinect.getIRMat();
		depth = kinect.getDepthMat();

		cv::imshow("color", color);
		cv::imshow("ir", ir);
		cv::imshow("depth", depth);

		if(cv::waitKey(3) >= 0) break;

		kinect.cleanFrames();
	}*/

    MjpgServer server(8081); // Start server on pot 8081
    server.setQuality(1); // Set jpeg quality to 1 (0 - 100)
    server.setResolution(640, 320); // Set stream resolution to 1280x720
    server.setFPS(30); // Set target fps to 15
    server.setMaxConnections(3);
    server.setName("Perception Server");
	server.attach(pullFrame);

    //kinect.setFPS(30);

	cv::namedWindow("rgb");

	kinect.attachRGB([](cv::Mat *rgbImage) {
		//std::cout << "NEW FRAME" << std::endl;
		//rgbMat = *rgbImage;
		cv::resize(*rgbImage, *rgbImage, cv::Size(720, 480));
		cv::flip(*rgbImage, *rgbImage, 1);

		cv::imshow("rgb", *rgbImage);
		cv::waitKey(1);
	});

	kinect.attachIR([](cv::Mat *irImage, kinect::DepthData *depthData) {
		cv::flip(*irImage, *irImage, 1);

		cv::Mat gray;
		cv::threshold(*irImage, gray, 100, 255, CV_THRESH_BINARY);

		//cv::medianBlur(gray, gray, 3);

		cv::imshow("ir", gray);
		cv::waitKey(1);

		std::cout << "Depth: " << depthData->getDepth(250, 250) << std::endl;
	});

	/*kinect.attachDepth([](kinect::DepthData *depthData) {
		//boost::thread([=]() {
		std::cout << "Depth: " << depthData->getDepth(250, 250) << std::endl;
		//});
		//cv::imshow("depth", *depthImage);
		//cv::waitKey(1);
	});*/

	kinect.startLoop();
    //server.run(); //Run stream forever (until fatal)

	while(1) {

	}

	cv::destroyAllWindows();
	Logger::Free();


	/*if (freenect2.enumerateDevices() == 0) {
		std::cout << "No device connected" << std::endl;
		return -1;
	}

	std::string serial = freenect2.getDefaultDeviceSerialNumber();
	std::cout << "SERIAL: " << serial << std::endl;

	if (!pipeline) {
		pipeline = new libfreenect2::OpenGLPacketPipeline();
	}

	if (pipeline) {
		std::cout << "Using pipeline!" << std::endl;
		dev = freenect2.openDevice(serial, pipeline);
	} else {
		std::cout << "Using standard serial!" << std::endl;
		dev = freenect2.openDevice(serial);
	}

	if (dev == 0) {
		std::cout << "Failed to open Kinectv2 device" << std::endl;
		return -1;
	}


	//Attach signal to shutdown loop
	signal(SIGINT, sigint_handler);
	protonect_shutdown = false; //Disable while loop below

	//Attach the multi frame listeners (For now one)
	libfreenect2::SyncMultiFrameListener listener(
			libfreenect2::Frame::Color | libfreenect2::Frame::Ir); // |*/
//	std::cout << "COLOR: " << libfreenect2::Frame::Depth << std::endl;

	/*//libfreenect2::Frame::Depth |
	//libfreenect2::Frame::Ir);
	//All mapped frames in one object
	libfreenect2::FrameMap frames;

	//Set the color grame listener to the MultiFrame Listener
	dev->setColorFrameListener(&listener);
	dev->setIrAndDepthFrameListener(&listener);

	dev->start(); //Start the listeners

	std::cout << "Device serial: " << dev->getSerialNumber() << std::endl;
	std::cout << "Device firmware: " << dev->getFirmwareVersion() << std::endl;

	//! [registration setup]
	//libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getColorCameraParams()); //new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
	//libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4); // check here (https://github.com/OpenKinect/libfreenect2/issues/337) and here (https://github.com/OpenKinect/libfreenect2/issues/464) why depth2rgb image should be bigger
	//! [registration setup]

	cv::Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;

	//Loop until shutdown
	while (!protonect_shutdown) {
		listener.waitForNewFrame(frames);
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
		//libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

		cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
		cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
		//cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);

		cv::imshow("rgb", rgbmat);
		//cv::imshow("ir", irmat / 4500.0f);
		//cv::imshow("depth", depthImg / 4500.0f);

		//! [registration]
		//registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
		//! [registration]

		//cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copytTo(depthmatUndistorted);
		//cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
		//cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);

		//cv::imshow("undistorted", depthmatUndistorted / 4500.0f);
		//cv::imshow("registered", rgbd);
		//cv::imshow("depth2RGB", rgbd2 / 4500.0f);

		int key = cv::waitKey(30);
		protonect_shutdown = protonect_shutdown
				|| (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

		//Destroy the frame
		listener.release(frames);
	}

	//Close the device
	dev->stop();
	dev->close();

	std::cout << "Finished perception!" << std::endl;
	*/
	return (0);
}
