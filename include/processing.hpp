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
 * @file processing.hpp
 * @date 01/23/2017
 * @brief Header to control the processing of Perception
 *
 * @details This header file contains all the definitions for the vision
 * processing of Perception and its components. The reason why the functions
 * are just in a namespace is to shorten the compile time by just the symbols
 * so we don't have to recompile main code every single time.
 */

//OPENCV INCLUDES
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/video.hpp>
#include <opencv2/photo.hpp>

//BOOST INCLUDES
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/function.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>

//STANDARD INCLUDES
#include <functional>
#include <memory>
#include <vector>

//NTCORE INCLUDES
#include <networktables/NetworkTable.h>

//PERCEPTION INCLUDES
#include "kinect.hpp"
#include "log.hpp"

#ifndef INCLUDE_PROCESSING_HPP_
#define INCLUDE_PROCESSING_HPP_

#define MUTEX_S_LOCK(X) boost::mutex::scoped_lock(X)

//Declare processing namespace
namespace processing {

	class Target {
	public:
		unsigned int area, sides, perim, x, y, depth;
		float width, height;
	};

	//All documentation provided in source file
	template<typename T>
	void inline PLOG(T, bool error = false);

	void kinectDepth2Mat(cv::Mat &, const kinect::DepthData &, const float);
	void kinectScalar(cv::Mat &, cv::Mat &);
	void preProcessing(cv::Mat, cv::Mat &);
	void processFrame(cv::Mat &, kinect::DepthData &, std::vector<Target> &);
	void settingsUpdate(std::shared_ptr<NetworkTable>);
} /* namespace processing */

#endif /* INCLUDE_PROCESSING_HPP_ */
