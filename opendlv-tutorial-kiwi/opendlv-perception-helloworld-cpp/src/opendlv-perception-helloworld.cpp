/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>

void MatchingMethod( int, void*, Mat img, Mat templPre, bool show_image);


int32_t main(int32_t argc, char **argv)
{
	int32_t retCode{1};
	auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
	if ((0 == commandlineArguments.count("cid")) ||
		(0 == commandlineArguments.count("name")) ||
		(0 == commandlineArguments.count("width")) ||
		(0 == commandlineArguments.count("height")))
	{
		std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
		std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
		std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
		std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
		std::cerr << "         --width:  width of the frame" << std::endl;
		std::cerr << "         --height: height of the frame" << std::endl;
		std::cerr << "Example: " << argv[0] << " --cid=112 --name=img.argb --width=640 --height=480 --verbose" << std::endl;
	}
	else
	{
		const std::string NAME{commandlineArguments["name"]};
		const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
		const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
		const bool VERBOSE{commandlineArguments.count("verbose") != 0};

		// Attach to the shared memory.
		std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
		if (sharedMemory && sharedMemory->valid())
		{
			std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

			// Interface to a running OpenDaVINCI session; here, you can send and receive messages.
			cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

			// Handler to receive distance readings (realized as C++ lambda).
			std::mutex distancesMutex;
			float front{0};
			float rear{0};
			float left{0};
			float right{0};
			auto onDistance = [&distancesMutex, &front, &rear, &left, &right](cluon::data::Envelope &&env) {
				auto senderStamp = env.senderStamp();
				// Now, we unpack the cluon::data::Envelope to get the desired DistanceReading.
				opendlv::proxy::DistanceReading dr = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(env));

				// Store distance readings.
				std::lock_guard<std::mutex> lck(distancesMutex);
				switch (senderStamp)
				{
				case 0:
					front = dr.distance();
					break;
				case 2:
					rear = dr.distance();
					break;
				case 1:
					left = dr.distance();
					break;
				case 3:
					right = dr.distance();
					break;
				}
			};
			// Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
			od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistance);

			Mat templ = imread("../imgs/template.png", IMREAD_GRAYSCALE);


			// Endless loop; end the program by pressing Ctrl-C.
			while (od4.isRunning())
			{
				 Mat img;

				// Wait for a notification of a new frame.
				sharedMemory->wait();

				// Lock the shared memory.
				sharedMemory->lock();
				{
					// Copy image into cvMat structure.
					// Be aware of that any code between lock/unlock is blocking
					// the camera to provide the next frame. Thus, any
					// computationally heavy algorithms should be placed outside
					// lock/unlock
					 Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
					img = wrapped.clone();
				}
				sharedMemory->unlock();

				cvtColor(img, img, cv::COLOR_BGR2GRAY);
				GaussianBlur( img, img, Size(3,3), 0, 0, BORDER_DEFAULT );

				// Canny Detection
				Mat canny;
				Canny(img, canny, 30, 100, 3);

				// Masking car and top half of picture
				rectangle(canny, Point(img.cols/5,img.rows-200), Point(img.cols*4/5,img.rows), Scalar(0,0,0), -1, 8);
				rectangle(canny, Point(0,0), Point(img.cols,img.rows*9/16), Scalar(0,0,0), -1, 8);

				MatchingMethod( 0, 0 ,canny, templ, VERBOSE);


				//  equalizeHist(img, img);
				/*Mat croppedFrame(img,  Rect(0, 270, 1280, 450));

				 Mat hsv;
				 cvtColor(croppedFrame, hsv,  COLOR_BGR2HSV);
				 Scalar hsvLow(110, 40, 40);
				 Scalar hsvHi(130, 255, 255);
				 Mat blueCones;
				 inRange(hsv, hsvLow, hsvHi, blueCones);

				uint32_t iterations{5};
				 Mat dilate;
				 dilate(blueCones, dilate,  Mat(),  Point(-1, -1), iterations, 1, 1);

				 Mat erode;
				 erode(dilate, erode,  Mat(),  Point(-1, -1), iterations, 1, 1);

				// Canny Detection
				 Mat canny;
                 Canny(croppedFrame, canny, 30, 90, 3);

                std::vector< Vec2f> lines;
                 HoughLines(canny, lines, 1, CV_PI/180, 150, 0, 0);

                //Draw the lines
                 Mat hough;
                 cvtColor(canny, hough,  COLOR_GRAY2BGR);
                for(size_t i = 0; i < lines.size(); i++)
                {
                    float rho = lines[i][0];
                    float theta = lines [i][1];
                    double a = cos(theta);
                    double b = sin(theta);
                    double x0 = a * rho;
                    double y0 = b * rho;

                     Point pt1;
                     Point pt2;
                    pt1.x = cvRound(x0 + 1000 * (-b));
                    pt1.y = cvRound(y0 + 1000 * a);
                    pt2.x = cvRound(x0 - 1000 * (-b));
                    pt2.y = cvRound(y0 - 1000 * a);
                     line(hough, pt1, pt2,  Scalar(0,255,255), 3,  LINE_AA);
                }*/

				// // Invert colors
				//  bitwise_not(img, img);

				// // Draw a red rectangle
				//  rectangle(img,  Point(50, 50),  Point(100, 100),  Scalar(0,0,255));

				// Display image.
				if (VERBOSE)
				{
					 imshow(sharedMemory->name().c_str(), erode);
					 waitKey(1);
				}

				////////////////////////////////////////////////////////////////
				// Do something with the distance readings if wanted.
				{
					std::lock_guard<std::mutex> lck(distancesMutex);
					std::cout << "front = " << front << ", "
							  << "rear = " << rear << ", "
							  << "left = " << left << ", "
							  << "right = " << right << "." << std::endl;
				}

				////////////////////////////////////////////////////////////////
				// Example for creating and sending a message to other microservices; can
				// be removed when not needed.
				opendlv::proxy::AngleReading ar;
				ar.angle(123.45f);
				od4.send(ar);

				////////////////////////////////////////////////////////////////
				// Steering and acceleration/decelration.
				//
				// Uncomment the following lines to steer; range: +38deg (left) .. -38deg (right).
				// Value groundSteeringRequest.groundSteering must be given in radians (DEG/180. * PI).
				//opendlv::proxy::GroundSteeringRequest gsr;
				//gsr.groundSteering(0);
				//od4.send(gsr);

				// Uncomment the following lines to accelerate/decelerate; range: +0.25 (forward) .. -1.0 (backwards).
				// Be careful!
				//opendlv::proxy::PedalPositionRequest ppr;
				//ppr.position(0);
				//od4.send(ppr);
			}
		}
		retCode = 0;
	}
	return retCode;
}


void MatchingMethod( int, void*, Mat img, Mat templPre, bool show_image)
{

	Mat templ;
	templPre.convertTo(templ, img.type());

	/// Source image to display
	Mat img_display;
	img.copyTo( img_display );

	Mat result;

	/// Do the Matching and Normalize
	matchTemplate( img, templ, result, 4 );
	normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

	
	float threshold = 0.8f;
	
	std::vector<Point> top_positions;

	// Find Positions of best matches
	for(int i = 0; i < result.rows -1; ++i)
	{
		for(int j = 0; j < result.cols - 1; ++j)
		{
			float pixel = result.at<float>(i, j);
			if (pixel >= threshold)
			{
				std::cout << "Pos: " << i << ", " << j << " Threshold: " << pixel << std::endl;
				top_positions.push_back(Point(j,i));
			}
		}
	}

	if (show_image)
	{

		// Draw rectangle around best matches.
		for (Point pos : top_positions)
		{
			rectangle( img_display, pos, Point( pos.x + templ.cols , pos.y + templ.rows ), Scalar(255,255,255),  2, 8, 0 );
			rectangle( result, pos, Point( pos.x + templ.cols , pos.y + templ.rows ), Scalar(255,255,255),  2, 8, 0);			
		}

		/// Localizing the best match with minMaxLoc
		double minVal; double maxVal; Point minLoc; Point maxLoc;
		Point matchLoc;
		minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
		matchLoc = maxLoc;

		/// Draw rectangle
		rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar(255,255,255), 2, 8, 0 );
		rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar(255,255,255), 2, 8, 0 );
		
		imshow("Image", img_display);
		waitKey(1);
	}
  return;
}