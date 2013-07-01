/*
 * PowerCubeArmUUISRC.h
 *
 * Hardware specific code for the PowerCube arm at UUISRC.
 * Extracted from old PowerCube class at best guess.
 */

#ifndef POWERCUBEARMUUISRC_H_
#define POWERCUBEARMUUISRC_H_

#include "SchunkProtocolArm.h"


class PowerCubeArmUUISRC: public SchunkProtocolArm {

#define GRIPPER_ID	(modules_count_-1)

public:
	virtual void init() {
		SchunkProtocolArm::init();

		ackAll();
		sleep(1);
		refAll();
		sleep(15);
		ackAll();
		sleep(1);
		refAll();
		sleep(15);
	}

	virtual int emergencyStopJoint(IDType id) {
		// ignoring the gripper?
		if(id == modules_count_-1) {
			ROS_WARN("emergency: ignoring module %d", id + id_offset_);
			return 0;
		}
		else {
			return SchunkProtocolArm::emergencyStopJoint(id);
		}
	}

	virtual int ackJoint(IDType id) {
		const SCHUNKMotionManipulator::ModuleConfig *moduleConfig;
		manipulator_.getModuleConfig(id + id_offset_, moduleConfig);
		if (moduleConfig->error_code == 217)
			// do not ack the non emergency stopped
			return 0;
		else
			return ackJoint(id);
	}

	virtual int refJoint(IDType id) {
		if(id == GRIPPER_ID) {
			const SCHUNKMotionManipulator::ModuleConfig *moduleConfig;
			manipulator_.getModuleConfig(id + id_offset_, moduleConfig);

			if(moduleConfig->status_flags.flags.referenced) {
				// dont ref the gripper anymore
				return 0;
			}
		}

		return ackJoint(id) + SchunkProtocolArm::refJoint(id);
	}

};

#endif /* POWERCUBEARMUUISRC_H_ */
