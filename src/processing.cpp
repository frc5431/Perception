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
 * @file processing.cpp
 * @date 01/23/2017
 * @brief Source file for the processing of Perception
 *
 * @details This source file contains all the definitions for the vision
 * processing of Perception and its components.
 */

#include "../include/processing.hpp"

/*
 * Everything labeled OLDSRC is for the kinect version of the code
 * which is currently deprecated, if other teams want to try it out
 * just uncomment the OLDSRC headed parts.
 */

//OLDSRC
/*
//Depth processing and checking
#define PROCESSING_DEPTH_Y_OFFSET	-20 //Subtract 20 pixels from target center
#define PROCESSING_DEPTH_MINIMUM	2000 //Minimum distance in millimeters to be from target
#define PROCESSING_DEPTH_MAXIMUM	4500 //Maximum distance in millimeters to be from target
*/


//Processing namespace (@TODO Split off common functions in separate source files to decrease compile time) 
namespace processing {

	//Mutex lock for settings
	boost::mutex proc_mutex;

	//Settings for processing
	unsigned int loop_time = PROCESSING_LOOP_TIME,
				 threshMinH = PROCESSING_MIN_THRESH_H,
				 threshMinS = PROCESSING_MIN_THRESH_S,
				 threshMinV = PROCESSING_MIN_THRESH_V,
				 threshMaxH = PROCESSING_MAX_THRESH_H,
				 threshMaxS = PROCESSING_MAX_THRESH_S,
				 threshMaxV = PROCESSING_MAX_THRESH_V,
				 medianBlue = PROCESSING_MEDIAN_BLUR,
				 dilationSize = PROCESSING_DILATION_SIZE,
				 minArea = PROCESSING_MIN_AREA,
				 maxArea = PROCESSING_MAX_AREA,
				 minSides = PROCESSING_MIN_SIDES,
				 maxSides = PROCESSING_MAX_SIDES,
				 minPerim = PROCESSING_MIN_PERIM,
				 maxPerim = PROCESSING_MAX_PERIM,
				 midYMin = PROCESSING_MID_MIN_Y,
				 midYMax = PROCESSING_MID_MAX_Y;
				//OLDSRC 
				//depthYOffset = PROCESSING_DEPTH_Y_OFFSET;

	float maxHeight = PROCESSING_MAX_HEIGHT,
		  widthRatio = PROCESSING_WIDTH_RATIO;


	template<typename T>
	void inline PLOG(T toLog, bool error) {
		Logger::Log(PROCESSING_LOG_TAG, SW(toLog), error);
	}

	/*void kinectDepth2Mat(cv::Mat &frame, const kinect::DepthData &depth, const float maxDepth) {
		frame = cv::Mat(depth.height, depth.width, CV_32FC1, depth.depthRaw) / maxDepth;
	}

	void kinectScalar(cv::Mat &frame, cv::Mat &newFrame) {
		frame.convertTo(newFrame, CV_8UC3);
	}*/

	void preProcessing(cv::Mat &frame, cv::Mat &newFrame) {
		//Inversed binary thresh. It's easier to view the target
		//cv::threshold(frame, frame, threshMin, threshMax, CV_THRESH_BINARY_INV);
		cv::cvtColor(frame, frame, CV_BGR2HSV);
		cv::inRange(frame, cv::Scalar(threshMinH, threshMinS, threshMinV), 
						cv::Scalar(threshMaxH, threshMaxS, threshMaxV), newFrame);

		//Blur the threshed image to remove random depth static
		cv::medianBlur(newFrame, newFrame, PROCESSING_MEDIAN_BLUR);

		//Smooth back the thicker parts of the image to crisp out the target
		cv::Mat dilater = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilationSize + 1,
					2 * dilationSize + 1),cv::Point(dilationSize, dilationSize));

		//Apply erode method
		cv::dilate(newFrame, newFrame, dilater);
	}

	void processFrame(cv::Mat &frame, std::vector<Target> &targets) {

		//Create the contour hierarchical status
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;

		//Find all contours on processed frame
		cv::findContours(frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

		unsigned int ind = 0; //Set contour count

		cv::Mat drawing = cv::Mat::zeros(frame.size(), CV_8UC3);

		//Loop through each contour found
		for(std::vector<cv::Point> contour : contours) {

			//Get the current contour area
			unsigned int contour_area = cv::contourArea(contour);

			//Check if the contour could be a possible target match with the area
			if(contour_area >= minArea && contour_area <= maxArea) {
				//Create an approx poly
				std::vector<cv::Point> approx;

				//Find the approx poly
				cv::approxPolyDP(contour, approx, 5, true);

				//Get the current poly side count
				unsigned int contour_sides = approx.size();

				//Check to see if the contour matches the correct side count
				if(contour_sides >= minSides && contour_sides <= maxSides) {

					//Get the contour perimeter size
					unsigned int contour_perim = cv::arcLength(contour, true);

					//Check to see if the perimeter length is the correct size
					if(contour_perim >= minPerim && contour_perim <= maxPerim) {

						//Get bounding rectangle for contour
						cv::Rect contour_bound = cv::boundingRect(contour);

						//Get contour bounding dimensions
						float contour_width = static_cast<float>(contour_bound.width);
						float contour_height = static_cast<float>(contour_bound.height);

						//Get the target width for a rectangle
						float targetX = contour_height * widthRatio;

						//Check to see if it's a valid target with the proper aspect ratio
						if(contour_width >= targetX && contour_height <= maxHeight) {

							//Get the moments of the contour
							cv::Moments mu = cv::moments(contour, false);

							//Get the mass center of the contour for both X and Y
							unsigned int contour_x = (mu.m10 / mu.m00);
							unsigned int contour_y = PROCESSING_CAMERA_HEIGHT - (mu.m01 / mu.m00);

							//Check to see if the contour is within our region of interest
							if(contour_y >= midYMin && contour_y <= midYMax) {

								//Get current contour depth
								//int contour_depth = depthData.getDepth(&frame, contour_x, (contour_y + depthYOffset));
								
								//Add contour to possible target object
								Target target;
								target.area = contour_area;
								target.width = contour_width;
								target.height = contour_height;
								target.perim = contour_perim;
								target.sides = contour_sides;
								target.x = contour_x;
								target.y = contour_y;
								//target.depth = contour_depth;

							cv::Scalar color = cv::Scalar(255, 0, 0);
							cv::drawContours(drawing, contours, ind, color, 2, 8, hierarchy, 0, cv::Point());
							cv::putText(drawing, boost::lexical_cast<std::string>(ind), cv::Point(contour_x, (mu.m01 / mu.m00)), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255));

								//Add current target to target calculation list
								targets.push_back(target);
							}

						}

					}

				}
			}

			cv::imshow("contours", drawing);

			ind++; //Add to current contour index
		}
	}

	void processTargets(std::vector<Target> targets) {
		unsigned int target_ind = 0;		

		for(Target target : targets) {

		

			target_ind++;
		}

	}

	void __settingsLoop(std::shared_ptr<NetworkTable> table) {
		while(1) {

			try {

				//Scoped mutex lock
				{
					MUTEX_S_LOCK(proc_mutex);
					PLOG("Pulling new settings!");
					loop_time = boost::lexical_cast<int>(table->GetNumber(PROCESSING_LOOP_TAG, PROCESSING_LOOP_TIME));
					PLOG(SW("LOOP TIME: ") + SW(loop_time));
				}

				boost::this_thread::sleep_for(boost::chrono::milliseconds(loop_time));
			} catch(const std::exception &err) {
				PLOG("Failed pulling new settings!");
			}
		}
	}

	void settingsUpdate(std::shared_ptr<NetworkTable> table) {
		PLOG("Starting the settings updater");
		boost::thread(processing::__settingsLoop, table);
	}

} /* namespace processing */
