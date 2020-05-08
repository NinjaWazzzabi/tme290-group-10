#include "cluon-complete.hpp"
#include "messages.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdint.h>
#include <iostream>
#include <memory>
#include <mutex>

using namespace cv;

void MatchingMethod( int, void* , Mat img, Mat templ, bool show_image );


int32_t main(int32_t , char **)
{
	int32_t retCode{1};

	const std::string NAME{"img.argb"};
	const uint32_t WIDTH{1280};
	const uint32_t HEIGHT{720};
	const bool VERBOSE{true};
	const uint32_t CID{111};


	// Attach to the shared memory.
	std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
	if (sharedMemory && sharedMemory->valid())
	{
		std::clog << " Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

		// Interface to a running OpenDaVINCI session; here, you can send and receive messages.
		cluon::OD4Session od4{CID};

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


		// Loading template image for matching
		Mat templ = imread("../imgs/template.png", IMREAD_GRAYSCALE);

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



			////////////////////////////////////////////////////////////////
			// Do something with the distance readings if wanted.
			{
				std::lock_guard<std::mutex> lck(distancesMutex);
				std::cout << "front = " << front << ", "
							<< "rear = " << rear << ", "
							<< "left = " << left << ", "
							<< "right = " << right << "." << std::endl;
			}
		}
	}
	retCode = 0;
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

	
	float threshold = 0.8;
	
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