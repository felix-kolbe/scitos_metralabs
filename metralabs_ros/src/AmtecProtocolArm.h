/*
 * AmtecProtocolArm.h
 *
 * Amtec protocol specific implementation of RobotArm class.
 */

#ifndef AMTECPROTOCOLARM_H_
#define AMTECPROTOCOLARM_H_

#include <MetraLabsBase.h>
#include <base/Errors.h>
// #include <hardware/AmtecManipulator.h>
#include "AmtecManipulator.h" // the modded version

#include "RobotArm.h"

using namespace MetraLabs::robotic::base;
using namespace MetraLabs::robotic::hardware;


class AmtecProtocolArm : public RobotArm {
public:
	AmtecProtocolArm(IDType id_offset = 20) : RobotArm(id_offset) {
	}

	virtual void init() {
		ROS_INFO("Searching AMTEC motion modules. Please wait...");
		Error tErr = manipulator_.initialize();
		if (tErr != OK) {
			ROS_FATAL("Failed to initialise the AMTEC Manipulator. Code: %s\n", getErrorString(tErr).c_str());
			exit(1);
		}
		modules_count_ = manipulator_.getModules().size();

		ROS_INFO("Found %d AMTEC modules", modules_count_);
	}

	virtual int emergencyStopJoint(IDType id) {
		ROS_WARN("emergency stopping module %d", id + id_offset_);
		return manipulator_.halt(id + id_offset_);
	}

	virtual int normalStopJoint(IDType id) {
		return manipulator_.softStop(id + id_offset_);
	}

	virtual int ackJoint(IDType id) {
		return manipulator_.reset(id + id_offset_);
	}

	virtual int refJoint(IDType id) {
		return manipulator_.ref(id + id_offset_);
	}


	virtual float getPosition(IDType id) {
		return manipulator_.getModules().at(id).status_pos;
	}
	virtual float getVelocity(IDType id) {
		return manipulator_.getModules().at(id).status_vel;
	}
	virtual float getMaxCurrent(IDType id) {
		return NAN;
	}

	virtual int setVelocity(IDType id, float velocity) {
		return manipulator_.setTargetVelocity(id + id_offset_, velocity);
	}
	virtual int setAcceleration(IDType id, float acceleration) {
		return manipulator_.setTargetAcceleration(id + id_offset_, acceleration);
	}
	virtual int setCurrent(IDType id, float current) {
		return ERR_NOT_SUPPORTED;
	}


	virtual int movePosition(IDType id, float position) {
		return manipulator_.execMotion(id + id_offset_, AmtecManipulatorMod::MOTION_FRAMP_MODE, position, 0);
	}
	virtual int movePositionDuration(IDType id, float position, uint16_t msecs) {
		return manipulator_.execMotion(id + id_offset_, AmtecManipulatorMod::MOTION_FSTEP_MODE, position, msecs);
	}
	virtual int moveVelocity(IDType id, float velocity) {
		return manipulator_.execMotion(id + id_offset_, AmtecManipulatorMod::MOTION_FVEL_MODE, velocity, 0);
	}



	virtual void getModuleStatus(IDType module_id, uint8_t& referenced, uint8_t& moving,
			uint8_t& prog_mode, uint8_t& warning, uint8_t& error, uint8_t& brake,
			uint8_t& move_end, uint8_t& pos_reached, uint8_t& error_code, float& current) {
		const AmtecManipulatorMod::ModuleConfig *module_config;
		manipulator_.getModuleConfig(module_id + id_offset_, module_config);

		brake = module_config->status_flags.flags.brake == 1;
		error = module_config->status_flags.flags.error == 1;
		move_end = module_config->status_flags.flags.ramp_end == 1; // TODO check if used correctly
		moving = module_config->status_flags.flags.motion == 1;
		pos_reached = module_config->status_flags.flags.brake == 0; // TODO // no better equivalent // really?
		prog_mode = 0; // no equivalent
		referenced = module_config->status_flags.flags.home_ok == 1;
		warning = module_config->status_flags.flags.cur_limit == 1;
		error_code = 0; // TODO no equivalent, check if needed by caller
		current = module_config->status_cur;
	}


	AmtecManipulatorMod& getManipulator() {
		return manipulator_;
	}

protected:
	AmtecManipulatorMod manipulator_;

};

#endif /* AMTECPROTOCOLARM_H_ */
