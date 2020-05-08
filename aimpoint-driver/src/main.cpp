#include "cluon-complete.hpp"
#include "messages.hpp"
#include <stdint.h>
#include <vector>
#include <iostream>
#include <mutex>
#include "cone_coder.hpp"

#define CID 111
#define FREQ 100

#define DRIVE 0
#define STOP 1

int32_t main(int32_t, char **)
{
	std::mutex m_external_data;
	uint32_t global_drive_state = STOP;
	std::vector<ConeMeasurment> global_cones;

	auto drive_state_listener{[&global_drive_state, &m_external_data](cluon::data::Envelope &&envelope) {
		auto drive_state_msg = cluon::extractMessage<opendlv::robo::DriveState>(std::move(envelope));
		{
			std::lock_guard<std::mutex> lock(m_external_data);
			global_drive_state = drive_state_msg.state();
		}
	}};

	auto cone_list_listener{[&global_cones, &m_external_data](cluon::data::Envelope &&envelope) {
		auto cones_message = cluon::extractMessage<opendlv::robo::ConeLocation>(std::move(envelope));
		std::vector<ConeMeasurment> cones = ConeCoder::decode(cones_message.data());
		{
			std::lock_guard<std::mutex> lock(m_external_data);
			global_cones.clear();
			global_cones = cones;
		}
	}};

	cluon::OD4Session od4{CID};
	od4.dataTrigger(opendlv::robo::DriveState::ID(), drive_state_listener);
	od4.dataTrigger(opendlv::robo::ConeLocation::ID(), cone_list_listener);

	auto aimpoint_runner{[]() -> bool {
		// Calculate aimpoint

		// Send steering value
		// Sent throttle value
		return true;
	}};
	od4.timeTrigger(FREQ, aimpoint_runner);
}