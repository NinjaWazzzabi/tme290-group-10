#include "cluon-complete.hpp"
#include "messages.hpp"
#include <stdint.h>
#include <vector>
#include <iostream>
#include <mutex>
#include "cone_location.hpp"
#include "serializer.hpp"
#include "kiwi_location.hpp"

#define CID 111
#define FREQ 10

#define T_LEFT  -20
#define T_RIGHT 60
#define T_WAIT_TIME 1000000 //One second

#define DRIVE 0
#define STOP 1

int32_t main(int32_t, char **)
{
	std::mutex m_external_data;
	std::vector<ConeLocation> global_cones;
	std::vector<KiwiLocation> global_kiwis;
	cluon::data::TimeStamp last_traffic_timestamp;

	auto cone_list_listener{[&global_cones, &m_external_data](cluon::data::Envelope &&envelope) {
		auto cones_message = cluon::extractMessage<opendlv::robo::ConeLocation>(std::move(envelope));
		std::vector<ConeLocation> cones = Serializer::decode<ConeLocation>(cones_message.data());
		{
			std::lock_guard<std::mutex> lock(m_external_data);
			global_cones.clear();
			global_cones = cones;
		}
	}};

	
	auto kiwi_list_listener{[&global_kiwis, &m_external_data](cluon::data::Envelope &&envelope) {
		auto kiwis_message = cluon::extractMessage<opendlv::robo::KiwiLocation>(std::move(envelope));
		std::vector<KiwiLocation> kiwis = Serializer::decode<KiwiLocation>(kiwis_message.data());
		{
			std::lock_guard<std::mutex> lock(m_external_data);
			global_kiwis.clear();
			global_kiwis = kiwis;
		}
	}};

	cluon::OD4Session od4{CID};
	od4.dataTrigger(opendlv::robo::ConeLocation::ID(), cone_list_listener);
	od4.dataTrigger(opendlv::robo::KiwiLocation::ID(), kiwi_list_listener);

	auto decision_runner{[&]() -> bool {

		// Copy variables to local scope
		std::vector<ConeLocation> cones;
		std::vector<KiwiLocation> kiwis;
		cluon::data::TimeStamp traffic_msg_timestamp;
		{
			std::lock_guard<std::mutex> lock(m_external_data);
			cones = global_cones;
			kiwis = global_kiwis;
		}

		// Check if intersection is reached
		bool intersection_reached = false;
		for (ConeLocation cone : cones)
		{
			if (cone.type() == CONE_INTERSECTION)
			{
				intersection_reached = true;
				break;
			}
		}

		if (intersection_reached)
		{
			// Check if traffic is present
			for (uint32_t i = 0; i < kiwis.size(); i++)
			{
				KiwiLocation kiwi = kiwis.at(i);
				if (T_LEFT < kiwi.relative_bearing() && kiwi.relative_bearing() < T_RIGHT)
				{
					last_traffic_timestamp = cluon::time::now();
					break;
				}

			}
		}

		float microseconds_since_last_traffic = cluon::time::deltaInMicroseconds(cluon::time::now(),last_traffic_timestamp);
		
		
		// Choose drive state
		uint32_t drive_state;
		if (microseconds_since_last_traffic < T_WAIT_TIME) 
		{
			drive_state = STOP;
		} else  
		{
			drive_state = DRIVE;
		}

		// Send drive state
		opendlv::robo::DriveState drive_state_msg;
		drive_state_msg.state(drive_state);
		od4.send(drive_state_msg);

		return true;
	}};
	od4.timeTrigger(FREQ, decision_runner);
}
