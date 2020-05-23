#include "cluon-complete.hpp"
#include "messages.hpp"
#include <stdint.h>
#include <vector>
#include <iostream>
#include <mutex>
#include "cone_coder.hpp"

#define CID 111
#define FREQ 10

#define T_LEFT  -20
#define T_RIGHT 60
#define T_TIME 0.5f

#define DRIVE 0
#define STOP 1

int32_t main(int32_t, char **)
{
	std::mutex m_external_data;
	std::vector<ConeMeasurment> global_cones;
	opendlv::robo::TrafficLocation global_traffic_msg;
	uint32_t global_traffic_msg_timestamp;

	auto cone_list_listener{[&global_cones, &m_external_data](cluon::data::Envelope &&envelope) {
		auto cones_message = cluon::extractMessage<opendlv::robo::ConeLocation>(std::move(envelope));
		std::vector<ConeMeasurment> cones = ConeCoder::decode(cones_message.data());
		{
			std::lock_guard<std::mutex> lock(m_external_data);
			global_cones.clear();
			global_cones = cones;
		}
	}};

	auto traffic_listener{[&global_traffic_msg, &global_traffic_msg_timestamp, &m_external_data](cluon::data::Envelope &&envelope) {
		auto traffic_msg = cluon::extractMessage<opendlv::robo::TrafficLocation>(std::move(envelope));
		{
			std::lock_guard<std::mutex> lock(m_external_data);
			global_traffic_msg_timestamp = cluon::time::toMicroseconds(cluon::time::now());
			global_traffic_msg = traffic_msg;
		}
	}};

	cluon::OD4Session od4{CID};
	od4.dataTrigger(opendlv::robo::ConeLocation::ID(), cone_list_listener);
	od4.dataTrigger(opendlv::robo::TrafficLocation::ID(), traffic_listener);

	auto decision_runner{[&]() -> bool {

		// Copy variables to local scope
		std::vector<ConeMeasurment> cones;
		opendlv::robo::TrafficLocation traffic_msg;
		uint32_t traffic_msg_timestamp = 0;
		{
			std::lock_guard<std::mutex> lock(m_external_data);
			cones = global_cones;
			traffic_msg = global_traffic_msg;
			traffic_msg_timestamp = global_traffic_msg_timestamp;
		}

		// Check if intersection is reached
		bool intersection_reached = false;
		for (ConeMeasurment cone : cones)
		{
			if (cone.type() == CONE_INTERSECTION)
			{
				intersection_reached = true;
				break;
			}
		}

		// Check if traffic is present
		float seconds_since_last_traffic_msg = float(cluon::time::toMicroseconds(cluon::time::now()) - traffic_msg_timestamp) / 1000000.0f;
		bool traffic_to_right = T_LEFT < traffic_msg.relative_bearing() && traffic_msg.relative_bearing() < T_RIGHT;

		// Choose drive state
		uint32_t drive_state;
		if (intersection_reached && traffic_to_right && seconds_since_last_traffic_msg < T_TIME)
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
