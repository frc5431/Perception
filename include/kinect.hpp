/*
 * Kinect.h
 *
 *  Created on: Jan 13, 2017
 *      Author: root
 */

#ifndef INCLUDE_KINECT_HPP_
#define INCLUDE_KINECT_HPP_

//KINECT INCLUDES
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <signal.h>

//kinect namespace
namespace kinect {

class Kinect {
public:
	Kinect();
	virtual ~Kinect();

	void loadKinect();
	void setFrames(int);
private:
	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = nullptr;
	libfreenect2::PacketPipeline *pipeline = nullptr;
	libfreenect2::SyncMultiFrameListener listener;
};

} /* namespace kinect */

#endif /* INCLUDE_KINECT_HPP_ */
