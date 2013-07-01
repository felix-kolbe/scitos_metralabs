/*
 * LWA3ArmUASHH.h
 *
 * Hardware specific code for the LWA3 arm at the Robot Vision Lab, UAS Hamburg.
 */

#ifndef LWA3ARMUASHH_H_
#define LWA3ARMUASHH_H_

#include "AmtecProtocolArm.h"


class LWA3ArmUASHH: public AmtecProtocolArm {

#define GRIPPER_ID	(modules_count_-1)

public:
	virtual void init() {
		AmtecProtocolArm::init();

		// ref the gripper
		ackJoint(GRIPPER_ID);
	}

	virtual int ackJoint(IDType id) {
		// the gripper has to be reset nicely
		const AmtecManipulator::ModuleConfig *moduleConfig;
		manipulator_.getModuleConfig(id + id_offset_, moduleConfig);

		if(id == GRIPPER_ID && moduleConfig->linear) {
			int retval = 0;
			if(!moduleConfig->status_flags.flags.home_ok) { // if not referenced, do so
				retval += refJoint(id); // start reference
				ros::Duration(4).sleep(); // wait to finish reference
			}
			if(id == GRIPPER_ID) { // set movable speeds in case too low ones were set
				retval += setAcceleration(id, 0.04);	// set gripper acceleration
				retval += setVelocity(id, 0.04);		// and velocity to a nice value
			}
			return retval;
		} else {
			return AmtecProtocolArm::ackJoint(id);
		}
	}

	virtual int refJoint(IDType id) {
		if(id == GRIPPER_ID) {
			setAcceleration(id, 0.04); // set gripper acceleration
			setVelocity(id, 0.04);	// and velocity to a nice value
		}

		return AmtecProtocolArm::refJoint(id);
	}

};

#endif /* LWA3ARMUASHH_H_ */
