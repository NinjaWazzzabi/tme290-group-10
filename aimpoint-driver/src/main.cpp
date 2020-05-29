#include "cluon-complete.hpp"
#include "messages.hpp"
#include <stdint.h>
#include <vector>
#include <iostream>
#include <mutex>
#include "cone_location.hpp"
#include "serializer.hpp"
#include "line.hpp"
#include "aimpoint-finder.hpp"

#define CID 111
#define FREQ 100

#define DRIVE 0
#define STOP 1

using namespace std;

/**
 * TODO: Will not handle intersection well, as cones on both sides are the same colour
 * Need to create a special handler for that. Perhaps, as the road is straight
 * at the intersection, we can simply "mark" cones as left or right depending
 * on their relative bearing.
*/

int32_t main(int32_t, char **)
{
	static constexpr double STEERING_P = 1.0;

	std::mutex m_external_data;
	uint32_t global_drive_state = STOP;
	std::vector<ConeLocation> global_cones;
	AimpointFinder aimpoint_finder;

	// TODO: listen to kiwi location messages

	auto drive_state_listener{[&global_drive_state, &m_external_data](cluon::data::Envelope &&envelope) {
		auto drive_state_msg = cluon::extractMessage<opendlv::robo::DriveState>(std::move(envelope));
		{
			std::lock_guard<std::mutex> lock(m_external_data);
			global_drive_state = drive_state_msg.state();
		}
	}};

	auto cone_list_listener{[&global_cones, &m_external_data](cluon::data::Envelope &&envelope) {
		auto cones_message = cluon::extractMessage<opendlv::robo::ConeLocation>(std::move(envelope));
		std::vector<ConeLocation> cones = Serializer::decode<ConeLocation>(cones_message.data());
		{
			std::lock_guard<std::mutex> lock(m_external_data);
			global_cones.clear();
			global_cones = cones;
		}
	}};

	cluon::OD4Session session{CID};
	session.dataTrigger(opendlv::robo::DriveState::ID(), drive_state_listener);
	session.dataTrigger(opendlv::robo::ConeLocation::ID(), cone_list_listener);

	auto aimpoint_runner{[&session, &global_cones, &aimpoint_finder, &m_external_data]() -> bool {
		vector<ConeLocation> cones;
		{
			std::lock_guard<std::mutex> lock(m_external_data);
			for (ConeLocation cone_location : global_cones)
			{
				cones.push_back(cone_location);
			}
		}

		Vector2d aimpoint = aimpoint_finder.find_aimpoint(cones);

		double aimpoint_angle_offset = aimpoint.x();

		double desired_steering = STEERING_P * aimpoint_angle_offset;

		opendlv::robo::Aimpoint aimpoint_message;
		opendlv::proxy::PedalPositionRequest throttle_request;
		opendlv::proxy::GroundSteeringRequest steering_request;

		aimpoint_message.x(aimpoint.x());
		aimpoint_message.y(aimpoint.y());
		aimpoint_message.steering_angle(desired_steering);
		// TODO: FIX, don't kill carr plz
		throttle_request.position(0.25);
		steering_request.groundSteering(desired_steering);

		session.send(throttle_request);
		session.send(steering_request);
		session.send(aimpoint_message);

		return true;
	}};
	session.timeTrigger(FREQ, aimpoint_runner);
}