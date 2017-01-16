/*
 * Kinect.h
 *
 *  Created on: Jan 13, 2017
 *      Author: root
 */

#ifndef INCLUDE_KINECT_HPP_
#define INCLUDE_KINECT_HPP_

#define KINECT_COLOR 1
#define KINECT_IR 2
#define KINECT_DEPTH 4

//STANDARD INCLUDES
#include <iostream>
#include <string>
#include <functional>
#include <algorithm>
#include <signal.h>
#include <stdint.h>

//KINECT INCLUDES
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

//OPENCV INCLUDES
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>

//BOOST INCLUDES
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/thread/mutex.hpp>

#define KINECT_LOGTAG "KINECT"
#define KINECT_FRAME_OFFSET 3 //Amount of frames to delay by on the

typedef std::function< void(cv::Mat *) > _attach_t;

//kinect namespace
namespace kinect {

struct DepthData {
	size_t width, height;
	unsigned char *depthRaw;

	int getDepth(int x, int y) {
		cv::Mat depth = (cv::Mat(height, width, CV_32FC1, depthRaw) / 4500.0f);
		return (static_cast<int>(depth.at<float>(x, y) * 4500.0f));
	}
};

class Kinect {
public:
	Kinect();
	virtual ~Kinect();
	void loadKinect();
	void setFrames(int);

	bool pullAll();
	void cleanFrames();

	void attachRGB(_attach_t);
	void attachIR(std::function< void(cv::Mat *, DepthData *) >);
	//void attachDepth(std::function< void(libfreenect2::Frame *, DepthData*) >);

	void setFPS(int);
	void startLoop();
	char getDepth(DepthData *, int, int);

	template<typename T>
	void KLOG(T, bool err=false);

protected:
	bool __is_init;
	int __fps;

	_attach_t rgbCallback; //, irCallback;
	std::function< void(cv::Mat *, DepthData *) > irCallback;//depthCallback;

	void __wait(long);
	long __FPS2millis(long);
	void __waitFPS(long fps);
	void __framePulling();
	void __rgb_callback(libfreenect2::Frame *);
	void __ir_callback(libfreenect2::Frame *, libfreenect2::Frame *);
	//void __depth_callback(libfreenect2::Frame *);

	boost::thread rgbThread, irThread, *kinectThread; //depthThread
	boost::mutex mutex;

	libfreenect2::FrameMap *frames;
	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = nullptr;
	libfreenect2::PacketPipeline *pipeline = nullptr;
	libfreenect2::SyncMultiFrameListener *listener;
};

} /* namespace kinect */
#endif /* INCLUDE_KINECT_HPP_ */
