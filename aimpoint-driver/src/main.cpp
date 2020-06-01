#include "cluon-complete.hpp"
#include "messages.hpp"
#include <stdint.h>
#include <vector>
#include <iostream>
#include <mutex>
#include "cone_location.hpp"
#include "kiwi_location.hpp"
#include "serializer.hpp"
#include "line.hpp"
#include "avg_aimpoint_finder.hpp"

#define CID 111
#define FREQ 100

#define DRIVE 0
#define STOP 1

using namespace std;

static constexpr double LOWPASS_WEIGHT = 0.05;
static constexpr uint32_t CAMERA_WIDTH = 1280;
static constexpr double CAMERA_FOV_X = 97.6;

static constexpr double MIN_DISTANCE = 0.4;
static constexpr double SPEED_P = 0.05;

double determine_throttle(std::vector<KiwiLocation> kiwis, double previous_throttle, double MAX_SPEED);
void send_control_request(double throttle, double steering, Vector2d aimpoint, cluon::OD4Session &session);

/**
 * TODO: Will not handle intersection well, as cones on both sides are the same colour
 * Need to create a special handler for that. Perhaps, as the road is straight
 * at the intersection, we can simply "mark" cones as left or right depending
 * on their relative bearing.
*/

int32_t main(int32_t, char **)
{
	static constexpr double STEERING_P = 0.02;
	const double MAX_SPEED = 0.25;

	std::mutex m_external_data;
	uint32_t global_drive_state = STOP;
	std::vector<ConeLocation> global_cones;
	std::vector<KiwiLocation> global_kiwis;
	AvgAimpointFinder aimpoint_finder = AvgAimpointFinder(CAMERA_WIDTH);
	Vector2d previous_aimpoint = {0.0, 0.0};
	double previous_throttle = 0;

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

	auto kiwi_list_listener{[&global_kiwis, &m_external_data](cluon::data::Envelope &&envelope) {
		auto kiwis_message = cluon::extractMessage<opendlv::robo::KiwiLocation>(std::move(envelope));
		std::vector<KiwiLocation> kiwis = Serializer::decode<KiwiLocation>(kiwis_message.data());
		{
			std::lock_guard<std::mutex> lock(m_external_data);
			global_kiwis.clear();
			global_kiwis = kiwis;
		}
	}};

	cluon::OD4Session session{CID};
	session.dataTrigger(opendlv::robo::DriveState::ID(), drive_state_listener);
	session.dataTrigger(opendlv::robo::ConeLocation::ID(), cone_list_listener);
	session.dataTrigger(opendlv::robo::KiwiLocation::ID(), kiwi_list_listener);

	auto aimpoint_runner{[&session, &global_drive_state, &global_cones, &global_kiwis, &previous_throttle, &MAX_SPEED, &aimpoint_finder, &previous_aimpoint, &m_external_data]() -> bool {
		// ─── LOAD VARAIBLES ──────────────────────────────────────────────
		vector<ConeLocation> cones;
		{
			std::lock_guard<std::mutex> lock(m_external_data);
			for (ConeLocation cone_location : global_cones)
			{
				cones.push_back(cone_location);
			}
		}

		vector<KiwiLocation> kiwis;
		{
			std::lock_guard<std::mutex> lock(m_external_data);
			for (KiwiLocation kiwi_location : global_kiwis)
			{
				kiwis.push_back(kiwi_location);
			}
		}

		uint32_t drive_state;
		{
			std::lock_guard<std::mutex> lock(m_external_data);
			drive_state = global_drive_state;
		}

		// ─── CALCULATE AIMPOINT ──────────────────────────────────────────
		Vector2d aimpoint = aimpoint_finder.find_aimpoint(cones, previous_aimpoint);
		Vector2d final_aimpoint = {
			(aimpoint.x() * LOWPASS_WEIGHT) + (1.0 - LOWPASS_WEIGHT) * previous_aimpoint.x(),
			(aimpoint.y() * LOWPASS_WEIGHT) + (1.0 - LOWPASS_WEIGHT) * previous_aimpoint.y()};

		double aimpoint_angle_offset = (final_aimpoint.x() - CAMERA_WIDTH / 2.0) * (CAMERA_FOV_X / CAMERA_WIDTH);
		double desired_steering = STEERING_P * aimpoint_angle_offset;

		// ─── HANDLE DRIVE STATE ──────────────────────────────────────────
		if (drive_state == DRIVE)
		{
			double desired_throttle = determine_throttle(kiwis, previous_throttle, MAX_SPEED);
			send_control_request(desired_throttle, desired_steering, final_aimpoint, session);

			previous_throttle = desired_throttle;
			previous_aimpoint = final_aimpoint;
		}
		else
		{
			double desired_throttle = 0.0;
			send_control_request(desired_throttle, desired_steering, aimpoint, session);
		}

		return true;
	}};
	session.timeTrigger(FREQ, aimpoint_runner);
}

double determine_throttle(std::vector<KiwiLocation> kiwis, double, double MAX_SPEED)
{
	if (kiwis.size() < 1)
	{
		return MAX_SPEED;
	}
	else
	{
		KiwiLocation kiwi = kiwis[0];

		if (kiwi.distance() > 0.7)
		{
			return MAX_SPEED;
		}
		else
		{
			double error_distance = kiwi.distance() - MIN_DISTANCE;
			return SPEED_P * error_distance;
		}
	}
}

void send_control_request(double throttle, double steering, Vector2d aimpoint, cluon::OD4Session &session)
{
	opendlv::robo::Aimpoint aimpoint_message;
	opendlv::proxy::PedalPositionRequest throttle_request;
	opendlv::proxy::GroundSteeringRequest steering_request;
	aimpoint_message.x(aimpoint.x());
	aimpoint_message.y(aimpoint.y());
	aimpoint_message.steering_angle(steering);
	throttle_request.position(throttle);
	steering_request.groundSteering(-steering);

	session.send(throttle_request);
	session.send(steering_request);
	session.send(aimpoint_message);
}
