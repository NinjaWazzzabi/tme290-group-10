#include "cluon-complete.hpp"
#include "messages.hpp"
#include "cone_location.hpp"
#include "serializer.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdint.h>
#include <iostream>
#include <memory>
#include <mutex>

using namespace cv;


int32_t main(int32_t , char **)
{
	int32_t retCode{1};

	const std::string NAME{"img.argb"};
	const uint32_t WIDTH{1280};
	const uint32_t HEIGHT{720};
	const uint16_t CID{111};
	std::mutex m_external_data;
	std::vector<ConeLocation> global_cones;
	double aimpoint_x = 0.0;
	double aimpoint_y = 0.0;
	std::deque<Mat> imgs;


	// Attach to the shared memory.
	std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
	if (sharedMemory && sharedMemory->valid())
	{
		std::clog << " Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

		// Interface to a running OpenDaVINCI session; here, you can send and receive messages.
		cluon::OD4Session od4{CID};


		auto cone_list_listener{[&global_cones, &m_external_data](cluon::data::Envelope &&envelope) {
			auto cones_message = cluon::extractMessage<opendlv::robo::ConeLocation>(std::move(envelope));
			std::vector<ConeLocation> cones = Serializer::decode<ConeLocation>(cones_message.data());
			{
				std::lock_guard<std::mutex> lock(m_external_data);
				global_cones.clear();
				global_cones = cones;
			}
		}};


		auto aimpoint_listener{[&aimpoint_x, &aimpoint_y, &m_external_data](cluon::data::Envelope &&envelope) {
			auto aimpoint_message = cluon::extractMessage<opendlv::robo::Aimpoint>(std::move(envelope));
			{
				std::lock_guard<std::mutex> lock(m_external_data);
				aimpoint_x = aimpoint_message.x();
				aimpoint_y = aimpoint_message.y();
			}
		}};
		

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
		od4.dataTrigger(opendlv::robo::Aimpoint::ID(), aimpoint_listener);
		od4.dataTrigger(opendlv::robo::ConeLocation::ID(), cone_list_listener);



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

			imgs.push_back(img);

			if(imgs.size() > 1 )
			{
				Mat frame;
				
				frame = imgs.at(0);
				imgs.pop_front();
			


				for (uint32_t i = 0; i < global_cones.size(); i++)
				{
					ConeLocation cone = global_cones[i];
					rectangle( frame, Point(cone.x(),cone.y()), Point(cone.x() + cone.w(),cone.y() + cone.h()), Scalar( 255,255,255 ), 2 );
				}
				
				circle(frame, Point(aimpoint_x, aimpoint_y), 20, Scalar(0, 125, 255), 5);

				imshow("debug", frame);
				waitKey(1);
			}
		}
	}
	retCode = 0;
	return retCode;
}