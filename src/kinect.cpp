/*
 * Kinect.cpp
 *
 *  Created on: Jan 13, 2017
 *      Author: root
 */

#include "../include/kinect.hpp"

namespace kinect {

Kinect::Kinect() {

}

Kinect::~Kinect() {
	// TODO Auto-generated destructor stub
}


//Open Kinect
void Kinect::loadKinect(){
	if (freenect2.enumerateDevices() == 0) {
			std::cout << "No device connected" << std::endl;
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
		}
		//Attach signal to shutdown loop
		signal(SIGINT, sigint_handler);
		protonect_shutdown = false; //Disable while loop below
}

void Kinect::setFrames(int frameType){
	this->listener = libfreenect2::SyncMultiFrameListener(frameType);
}





} /* namespace kinect */
