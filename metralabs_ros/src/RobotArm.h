/*
 * RobotArm.h
 *
 * Common interface for robot arms controlled via MetraLabs API.
 */

#ifndef ROBOTARM_H_
#define ROBOTARM_H_

#include <boost/noncopyable.hpp>

#include <ros/ros.h>


class RobotArm : private boost::noncopyable {

public:
	typedef uint_fast16_t IDType; // avoid char-typedef'ed uint8_t


	RobotArm(IDType id_offset = 0)
		: id_offset_(id_offset),
		  modules_count_(0)
	{
	}

	virtual ~RobotArm() {
	}

	virtual void init() = 0;


	virtual int emergencyStopJoint(IDType id) = 0;
	virtual void emergencyStopAll() {
		for (IDType id = 0; id < modules_count_; ++id)
			emergencyStopJoint(id);
	}

	virtual int normalStopJoint(IDType id) = 0;
	virtual void normalStopAll() {
		for (IDType id = 0; id < modules_count_; ++id)
			normalStopJoint(id);
	}

	virtual int ackJoint(IDType id) = 0;
	virtual void ackAll() {
		for (IDType id = 0; id < modules_count_; ++id)
			ackJoint(id);
	}

	virtual int refJoint(IDType id) = 0;
	virtual void refAll() {
		for (IDType id = 0; id < modules_count_; ++id)
			refJoint(id);
	}


	virtual float getPosition(IDType id) = 0;
	virtual float getVelocity(IDType id) = 0;
	virtual float getMaxCurrent(IDType id) = 0;


	virtual int setVelocity(IDType id, float velocity) = 0;
	virtual int setAcceleration(IDType id, float acceleration) = 0;
	virtual int setCurrent(IDType id, float current) = 0;

	virtual void setCurrentsToMax() {
		for (IDType id = 0; id < modules_count_; ++id)
			setCurrent(id, getMaxCurrent(id));
	}

	virtual int movePosition(IDType id, float position) = 0;
	virtual int movePositionDuration(IDType id, float position, uint16_t msecs) = 0;
	virtual int moveVelocity(IDType id, float v) = 0;



	// TODO maybe replace those uint8_t with bool
	virtual void getModuleStatus(IDType module_id, uint8_t& referenced, uint8_t& moving,
			uint8_t& prog_mode, uint8_t& warning, uint8_t& error, uint8_t& brake,
			uint8_t& move_end, uint8_t& pos_reached, uint8_t& error_code, float& current) = 0;


	unsigned int getModulesCount() const {
		return modules_count_;
	}

protected:
	/* Offset used to for mapping joint zero-based joint index on ROS side to offset-based index on hardware side.
	 * Therefore it's only to be added when passing an index to a lower level method. */
	IDType id_offset_;

	unsigned int modules_count_;
};


#endif /* ROBOTARM_H_ */
