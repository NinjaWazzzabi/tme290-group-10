#include "cluon-complete.hpp"
#include "messages.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdint.h>
#include <iostream>
#include <memory>
#include <mutex>

#include "cone_location.hpp"
#include "serializer.hpp"

using namespace cv;

void MatchingMethod( int, void* , Mat img, Mat templ, bool show_image );

ConeLocation extractConeData(Rect bbox, uint32_t type, Point CAMERA_POS, double TO_DEGREES, double cropped_top,uint32_t HEIGHT);

int32_t main(int32_t , char **)
{
	int32_t retCode{1};

	const double TO_DEGREES{180 / 3.141592653589793};
	const std::string NAME{"img.argb"};
	const uint32_t WIDTH{1280};
	const uint32_t HEIGHT{720};
	const bool VERBOSE{false};
	const uint16_t CID{111};
	const Point CAMERA_POS = Point(WIDTH/2, HEIGHT);


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
		//Mat templ = imread("../imgs/template.png", IMREAD_GRAYSCALE);

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


			Rect myROI(0, img.rows*9.4/16, img.cols, img.rows*4.2/16);
			Mat croppedImage = img(myROI);

			Mat red_prep;
			bitwise_not(croppedImage,red_prep);

			Mat hsv_inv;
			cvtColor(red_prep, hsv_inv,cv::COLOR_BGR2HSV);


			Mat  hsv;
			cvtColor(croppedImage , hsv , cv::COLOR_BGR2HSV );



			//Blue filter
			Scalar  bluLow(100,110,30);
			Scalar  bluHi(140,255,255);
			Mat blue_mask;
			inRange(hsv , bluLow , bluHi , blue_mask);

			//Yellow filter
			Scalar  yellowLow(10,60,60);
			Scalar  yellowHi(41,255,255);
			Mat yellow_mask;
			inRange(hsv,yellowLow,yellowHi, yellow_mask);



			//Red filter
			Scalar  redLow(75, 100, 100);
			Scalar  redHi(95,255,255);
			Mat red_mask;
			inRange(hsv_inv , redLow , redHi , red_mask);
			


			


			// Remove "holes" in yellow cones
			Mat  yellow_dilate;
			uint32_t  iterations = 5;
			cv::dilate(yellow_mask , yellow_dilate , Mat(), Point(-1,  -1), iterations , 1, 1);
			Mat  yellow_erode;
			cv::erode(yellow_dilate , yellow_erode , Mat(), Point(-1,  -1), iterations, 1, 1);
			yellow_mask = yellow_erode;
			
			
			// Remove "holes" in red cones
			Mat  red_dilate;
			cv::dilate(red_mask , red_dilate , Mat(), Point(-1,  -1), iterations , 1, 1);
			Mat  red_erode;
			cv::erode(red_dilate , red_erode , Mat(), Point(-1,  -1), iterations, 1, 1);
			red_mask = red_erode;
			

			// Remove "holes" in blue cones
			Mat  blue_dilate;
			cv::dilate(blue_mask , blue_dilate , Mat(), Point(-1,  -1), iterations , 1, 1);
			Mat  blue_erode;
			cv::erode(blue_dilate , blue_erode , Mat(), Point(-1,  -1), iterations, 1, 1);
			blue_mask = blue_erode;


			//Cone filter
			Mat cones_mask;
			bitwise_or(blue_mask,yellow_mask,cones_mask);

			bitwise_or(cones_mask,red_mask,cones_mask);



			// Vector to store cone positions
			std::vector<ConeLocation> cone_data;


			// Find Yellow Contours
			std::vector<std::vector<Point>> yellow_contours;
			std::vector<Vec4i> yellow_hierarchy;
			findContours( yellow_mask, yellow_contours, yellow_hierarchy, cv::RETR_EXTERNAL , cv::CHAIN_APPROX_SIMPLE, Point(0, 0) );
			
			for( int i = 0; i < (int) yellow_contours.size(); i++ )
			{
				Rect yellow_bounding_rect = boundingRect( yellow_contours.at(i));
				if (yellow_bounding_rect.width * 1.1 <=  yellow_bounding_rect.height)
				{
					if (yellow_bounding_rect.width * yellow_bounding_rect.height > 250 && yellow_bounding_rect.width * yellow_bounding_rect.height < 6000 )
					{
						cone_data.push_back(extractConeData(yellow_bounding_rect, CONE_LEFT,CAMERA_POS,TO_DEGREES,img.rows*9.4/16, HEIGHT));
						rectangle( croppedImage, yellow_bounding_rect.tl(), yellow_bounding_rect.br(), Scalar( 255,255,255 ), 2 );
					}
				}
			}

			
			// Find red Contours
			std::vector<std::vector<Point>> red_contours;
			std::vector<Vec4i> red_hierarchy;
			findContours( red_mask, red_contours, red_hierarchy, cv::RETR_EXTERNAL , cv::CHAIN_APPROX_SIMPLE, Point(0, 0) );
			
			for( int i = 0; i < (int) red_contours.size(); i++ )
			{
				Rect red_bounding_rect = boundingRect( red_contours.at(i));
				if (red_bounding_rect.width * 1.1 <=  red_bounding_rect.height)
				{
					if (red_bounding_rect.width * red_bounding_rect.height > 250 && red_bounding_rect.width * red_bounding_rect.height < 6000 )
					{
						cone_data.push_back(extractConeData(red_bounding_rect, CONE_INTERSECTION,CAMERA_POS,TO_DEGREES,img.rows*9.4/16, HEIGHT));
						rectangle( croppedImage, red_bounding_rect.tl(), red_bounding_rect.br(), Scalar( 255,255,255 ), 3 );
					}
				}
			}



			// Find Blue Contours
			std::vector<std::vector<Point>> blue_contours;
			std::vector<Vec4i> blue_hierarchy;
			findContours( blue_mask, blue_contours, blue_hierarchy, cv::RETR_EXTERNAL , cv::CHAIN_APPROX_SIMPLE, Point(0, 0) );

			for( int i = 0; i < (int) blue_contours.size(); i++ )
			{
				Rect blue_bounding_rect = boundingRect( blue_contours.at(i));
				if (blue_bounding_rect.width * 1.1 <=  blue_bounding_rect.height)
				{
					if (blue_bounding_rect.width * blue_bounding_rect.height > 250 && blue_bounding_rect.width * blue_bounding_rect.height < 6000 )
					{
						cone_data.push_back(extractConeData(blue_bounding_rect, CONE_RIGHT,CAMERA_POS,TO_DEGREES,img.rows*9.4/16, HEIGHT));
						rectangle( croppedImage, blue_bounding_rect.tl(), blue_bounding_rect.br(), Scalar( 255,255,255 ), 2 );
					}
				}
			}

			// Send cone data
			opendlv::robo::ConeLocation cl;
			cl.data(Serializer::encode(cone_data)); 
			od4.send(cl);

			if (VERBOSE)
			{	
				if (cone_data.size() > 0)
				{
					for (ConeLocation c : cone_data)
					{
						std::cout << "X:  " << c.x() << "  y:  " << c.y() <<  "  Bearing:  " << c.relative_bearing() << "  DISTANCE:  " << c.distance() << "  Type:  " << c.type() <<  std::endl;
					}
				}
			}

		}
	}
	retCode = 0;
	return retCode;
}

ConeLocation extractConeData(Rect bbox, uint32_t type, Point CAMERA_POS, double TO_DEGREES, double cropped_top,uint32_t HEIGHT)
{
	double x = bbox.x;
	double y = bbox.y + cropped_top;
	Point center = Point(x + bbox.width/2, HEIGHT - (y + bbox.height/2) ); 
	float angle = 90 - atan2(center.y, center.x - CAMERA_POS.x) * TO_DEGREES;
	return ConeLocation(x,y,bbox.width,bbox.height, angle, center.y, type);
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

		/// Find the best match with minMaxLoc
		double minVal; double maxVal; Point minLoc; Point maxLoc;
		Point matchLoc;
		minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
		matchLoc = maxLoc;

		/// Draw rectangle
		rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar(255,255,255), 2, 8, 0 );
		rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar(255,255,255), 2, 8, 0 );
		
	}
  return;
}