/*
 * SchunkProtocolArm.h
 *
 * Schunk protocol specific implementation of RobotArm class.
 */

#ifndef SCHUNKPROTOCOLARM_H_
#define SCHUNKPROTOCOLARM_H_

#include <MetraLabsBase.h>
#include <base/Errors.h>
#include <hardware/SCHUNKMotionManipulator.h>

#include "RobotArm.h"

using namespace MetraLabs::robotic::base;
using namespace MetraLabs::robotic::hardware;

/* these are used here as the Schunk protocol uses degrees */
#define DEG_TO_RAD(d)	((d)*M_PI/180.0)
#define RAD_TO_DEG(r)	((r)*180.0/M_PI)


class SchunkProtocolArm : public RobotArm {
public:
	SchunkProtocolArm(IDType id_offset = 100) : RobotArm(id_offset) {
	}

	virtual void init() {
		ROS_INFO("Searching SCHUNK motion modules. Please wait...");
		Error tErr = manipulator_.initialize();
		if (tErr != OK) {
			ROS_FATAL("Failed to initialise the SCHUNK Manipulator. Code: %s\n", getErrorString(tErr).c_str());
			exit(1);
		}
		modules_count_ = manipulator_.getModules().size();

		ROS_INFO("Found %d SCHUNK modules", modules_count_);
	}

	virtual int emergencyStopJoint(IDType id) {
		ROS_WARN("emergency stopping module %d", id + id_offset_);
		return manipulator_.emergencyStop(id + id_offset_);
	}

	virtual int normalStopJoint(IDType id) {
		return manipulator_.stop(id + id_offset_);
	}

	virtual int ackJoint(IDType id) {
		return manipulator_.ack(id + id_offset_);
	}

	virtual int refJoint(IDType id) {
		return manipulator_.ref(id + id_offset_);
	}


	virtual float getPosition(IDType id) {
		return DEG_TO_RAD(manipulator_.getModules().at(id).status_pos);
	}
	virtual float getVelocity(IDType id) {
		return DEG_TO_RAD(NAN);
	}
	virtual float getMaxCurrent(IDType id) {
		return manipulator_.getModules().at(id).max_current;
	}


	virtual int setVelocity(IDType id, float velocity) {
		return manipulator_.setTargetVelocity(id + id_offset_, RAD_TO_DEG(velocity));
	}
	virtual int setAcceleration(IDType id, float acceleration) {
		return manipulator_.setTargetAcceleration(id + id_offset_, RAD_TO_DEG(acceleration));
	}
	virtual int setCurrent(IDType id, float current) {
		return manipulator_.setTargetCurrent(id + id_offset_, current);
	}


	virtual int movePosition(IDType id, float position) {
		return manipulator_.movePos(id + id_offset_, RAD_TO_DEG(position));
	}
	virtual int movePositionDuration(IDType id, float position, uint16_t msecs) {
		return ERR_NOT_SUPPORTED;
	}
	virtual int moveVelocity(IDType id, float velocity) {
		return manipulator_.moveVel(id + id_offset_, RAD_TO_DEG(velocity));
	}



	virtual void getModuleStatus(IDType module_id, uint8_t& referenced, uint8_t& moving,
			uint8_t& prog_mode, uint8_t& warning, uint8_t& error, uint8_t& brake,
			uint8_t& move_end, uint8_t& pos_reached, uint8_t& error_code, float& current) {
		const SCHUNKMotionManipulator::ModuleConfig *module_config;
		manipulator_.getModuleConfig(module_id + id_offset_, module_config);

		brake = module_config->status_flags.flags.brake == 1;
		error = module_config->status_flags.flags.error == 1;
		move_end = module_config->status_flags.flags.move_end == 1;
		moving = module_config->status_flags.flags.moving == 1;
		pos_reached = module_config->status_flags.flags.pos_reached == 1;
		prog_mode = module_config->status_flags.flags.prog_mode == 1;
		referenced = module_config->status_flags.flags.referenced == 1;
		warning = module_config->status_flags.flags.warning == 1;
		error_code = module_config->error_code;
		current = module_config->norm_current;
	}


	const SCHUNKMotionManipulator& getManipulator() const {
		return manipulator_;
	}

protected:
	SCHUNKMotionManipulator manipulator_;

};

#endif /* SCHUNKPROTOCOLARM_H_ */
