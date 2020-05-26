#include "cluon-complete.hpp"
#include "messages.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect.hpp>

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
	CascadeClassifier face_cascade;
	if(!face_cascade.load( "/usr/bin/cascade.xml" ))
		std::cout << "ERROR in loading Cascade" << std::endl;


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


			//Rect myROI(0, img.rows*8/16, img.cols, img.rows*4.2/16);
			//Mat croppedImage = img(myROI);



			Mat  gray;
			cv::cvtColor(img , gray , cv::COLOR_BGR2GRAY );

			std::vector<Rect> kiwis;
			face_cascade.detectMultiScale( gray, kiwis, 1.1, 2, 0, Size(60, 40), Size(300,280) );

			for( size_t i = 0; i < kiwis.size(); i++ )
			{
				cv::rectangle(img, Point(kiwis[i].x,kiwis[i].y), Point(kiwis[i].x + kiwis[i].width, kiwis[i].y + kiwis[i].height), Scalar(255,0,0));
			}


			imshow("kiwis", img);
			waitKey(1);

		}
	}
	retCode = 0;
	return retCode;
}
