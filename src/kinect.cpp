/*
 * Kinect.cpp
 *
 *  Created on: Jan 13, 2017
 *      Author: root
 */

#include "../include/kinect.hpp"
#include "../include/log.hpp"

namespace kinect {

Kinect::Kinect() {
	KLOG("Initializing Kinect device");

	this->__fps = 30;
	this->protonect_shutdown = true;
	this->__is_init = false;

	this->listener = nullptr;
	this->frames = new libfreenect2::FrameMap;
}

Kinect::~Kinect() {
	this->cleanFrames();

	KLOG("Cleaning up Kinect");

	this->__is_init = false;

	delete this->frames;
	delete this->listener;

	this->protonect_shutdown = true;

	if(dev != nullptr) {
		dev->stop();
		dev->close();
	}
}

//Open Kinect
void Kinect::loadKinect(){
	if (this->freenect2.enumerateDevices() == 0) {
			std::cout << "No device connected" << std::endl;
		}

		std::string serial = this->freenect2.getDefaultDeviceSerialNumber();
		KLOG("Device Serial: " + serial);

		if (!this->pipeline) {
			this->pipeline = new libfreenect2::OpenGLPacketPipeline();
		}

		if (this->pipeline) {
			KLOG("Using pipeline!");
			this->dev = freenect2.openDevice(serial, this->pipeline);
		} else {
			KLOG("Using standard serial!");
			this->dev = freenect2.openDevice(serial);
		}

		if (this->dev == 0) {
			KLOG("Failed to open Kinectv2 device");
		}
		//Attach signal to shutdown loop
		this->protonect_shutdown = false; //Disable while loop below
}

void Kinect::setFrames(int frameType){
	if(this->__is_init) this->listener = nullptr;

	KLOG("Setting Kinect frames");

	this->listener = new libfreenect2::SyncMultiFrameListener(frameType);

	this->__is_init = true;

	//Set the color grame listener to the MultiFrame Listener
	this->dev->setColorFrameListener(this->listener);
	if(frameType > KINECT_COLOR) {
		this->dev->setIrAndDepthFrameListener(this->listener);
	}
	this->dev->start();

	KLOG("Started the listener");
}

bool Kinect::pullAll() {
	if(this->__is_init) {
		this->listener->waitForNewFrame(*this->frames);
		return (true);
	}
	KLOG("Kinect not initialized!");
	return (false);
}

void Kinect::cleanFrames() {
	if(!this->__is_init) {
		KLOG("Kinect not initialized!");
		return;
	}
	this->listener->release(*this->frames);
}

template<typename T>
void Kinect::KLOG(T toLog, bool err) {
	Logger::Log(SW(KINECT_LOGTAG).c_str(), SW(toLog), err);
}

/*
 * KINECT FRAME LISTENERS
 *
 */

void Kinect::attachRGB(_attach_t callback) {
	this->rgbCallback = callback;
}

void Kinect::attachIR(_attach_t callback) {
	this->irCallback = callback;
}

void Kinect::attachDepth(_attach_t callback) {
	this->depthCallback = callback;
}

void Kinect::__wait(const long millis) {
	boost::this_thread::sleep_for(boost::chrono::milliseconds(millis));
}

void Kinect::__rgb_callback() {
	KLOG("Starting rgb callback loop");
	while(1) {
		try {
			int fps;

			this->mutex.lock();
			libfreenect2::Frame *rgb = this->rgbFrame;
			fps = this->__fps;
			this->mutex.unlock();

			rgbCallback(cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data));

			__waitFPS(fps);
		} catch(const std::exception &err) {
			KLOG(SW("Failed getting depth frame: ") + err.what(), true);
		}
	}
}

void Kinect::__ir_callback() {
	KLOG("Starting ir callback loop");
	while(1) {
		try {
			int fps;

			this->mutex.lock();
			libfreenect2::Frame *ir = this->irFrame;
			fps = this->__fps;
			this->mutex.unlock();

			irCallback(cv::Mat(ir->height, ir->width, CV_32FC1, ir->data) / 4500.0f);

			__waitFPS(fps);
		} catch(const std::exception &err) {
			KLOG(SW("Failed getting depth frame: ") + err.what(), true);
		}
	}
}


void Kinect::__depth_callback() {
	KLOG("Starting depth callback loop");
	while(1) {
		try {
			int fps;

			this->mutex.lock();
			cv::Mat tempDepthMat = this->rgbMat;
//			libfreenect2::Frame *depth = this->depthFrame;
			fps = this->__fps;
			this->mutex.unlock();

//			depthCallback(cv::Mat(depth->height, depth->width, CV_32FC1, depth->data) / 4500.0f);
			depthCallback(tempDepthMat / 4500.0f);

			__waitFPS(fps);
		} catch(const std::exception &err) {
			KLOG(SW("Failed getting depth frame: ") + err.what(), true);
		}
	}
}

void Kinect::__framePulling() {
	KLOG("Started the frame updates and pulling!");
	while(1) {
		try {
			this->mutex.lock();
			this->pullAll(); //Pull the new frame

			this->rgbFrame = ((*this->frames)[libfreenect2::Frame::Color]);
			this->irFrame = ((*this->frames)[libfreenect2::Frame::Ir]);
//			this->depthFrame = ((*this->frames)[libfreenect2::Frame::Depth]);
			libfreenect2::Frame *depth = this->depthFrame;
			this->rgbMat = cv::Mat(depth->height, depth->width, CV_32FC1, depth->data);
			this->depthFrame = ((*this->frames)[this->rgbMat]);

			this->cleanFrames();

			int fps = this->__fps + KINECT_FRAME_OFFSET;
			this->mutex.unlock();

			__waitFPS(fps);
		} catch(const std::exception &err) {
			KLOG(SW("Failed pulling new frame from the Kinect: ") + err.what(), true);
		}
	}
}

void Kinect::setFPS(int fps) {
	this->__fps = fps;
}

long inline Kinect::__FPS2millis(long fps) {
	return ((1 / fps) * 1000);
}

void inline Kinect::__waitFPS(long fps) {
	__wait(__FPS2millis(fps));
}

void Kinect::startLoop() {
	this->rgbThread = new boost::thread(boost::bind(&Kinect::__rgb_callback, this));
	this->irThread = new boost::thread(boost::bind(&Kinect::__ir_callback, this));
	this->depthThread = new boost::thread(boost::bind(&Kinect::__depth_callback, this));

	this->kinectThread = new boost::thread(boost::bind(&Kinect::__framePulling, this));
}



/*void Kinect::ourWaitKey(int wait){
	int key = cv::waitKey(wait);
	protonect_shutdown = protonect_shutdown|| (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape
	this->listener->release(*this->frames);
}

void Kinect::showAll(){
	cv::imshow("rgb", );
	cv::imshow("ir", irmat / 4500.0f);
	cv::imshow("depth", depthmat / 4500.0f);
}

void Kinect::showRGB(){
	cv::imshow("rgb",rgbmat);
}

void Kinect::showIR(){
	cv::imshow("ir",irmat);
}

void Kinect::showDepth(){
	cv::imshow("depth",depthmat);
}*/



} /* namespace kinect */
