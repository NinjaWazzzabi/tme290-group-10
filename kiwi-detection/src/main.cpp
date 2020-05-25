#include "cluon-complete.hpp"
#include "messages.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdint.h>
#include <iostream>
#include <memory>
#include <mutex>

using namespace cv;

int32_t main(int32_t, char **)
{
int32_t retCode{1};

	//const double TO_DEGREES{180 / 3.141592653589793};
	const std::string NAME{"img.argb"};
	const uint32_t WIDTH{1280};
	const uint32_t HEIGHT{720};
	//const bool VERBOSE{true};
	const uint16_t CID{111};
	//const Point CAMERA_POS = Point(WIDTH/2, HEIGHT);


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


			Rect myROI(0, img.rows*8/16, img.cols, img.rows*4.2/16);
			Mat croppedImage = img(myROI);


			Mat  hsv;
			cvtColor(croppedImage , hsv , cv::COLOR_BGR2HSV );

		
			Mat canny_out;
			Canny( hsv, canny_out, 30, 90, 3 );

			imshow("edges", canny_out);
			waitKey(1);

		}
	}
	retCode = 0;
	return retCode;
}
