#ifndef SCHUNKSERVER_H_
#define SCHUNKSERVER_H_

#include <boost/thread.hpp>
#include <boost/noncopyable.hpp>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <urdf/model.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Int8.h>

#include <metralabs_msgs/IDAndFloat.h>
#include <metralabs_msgs/SchunkStatus.h>

#include <sensor_msgs/JointState.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include "RobotArm.h"
#include "AmtecProtocolArm.h"
//#include "SchunkProtocolArm.h"
#include "LWA3ArmUASHH.h"
//#include "PowerCubeArmUUISRC.h"

#include <util/MTime.h>
#include <util/MTimeSpan.h>

using namespace std;


#define DEG_TO_RAD(d)	((d)*M_PI/180.0)
#define RAD_TO_DEG(r)	((r)*180.0/M_PI)

#define SLOWEST_REASONABLE_JOINT_SPEED	0.015f // the slowest reasonable speed for our joints (IMHO)
#define TOO_SLOW	0.002f //



class TrajectoryExecuter : private boost::noncopyable {
public:
	TrajectoryExecuter(ros::NodeHandle& nh) :
		nh_(nh),
		as_(NULL),
		arm_(NULL)
	{
		run_ = false;
		waiting_ = false;

		// make sure isnanf is working here and now
		ROS_ASSERT(isnanf(0 / (float)0));
		ROS_ASSERT(isnanf(-0 / (float)0));
	}

	void init(RobotArm* arm, std::map<std::string, RobotArm::IDType>& joints_name_to_number_map) {

		arm_ = arm;
		joints_name_to_number_map_ = joints_name_to_number_map;

		state_publisher_ = nh_.advertise<control_msgs::JointTrajectoryControllerState>("trajectory_state", 5);

		std::map<std::string, RobotArm::IDType>::iterator it;
		for (it = joints_name_to_number_map_.begin(); it != joints_name_to_number_map_.end(); ++it) {
			state_msg_.joint_names.push_back(it->first);
		}
		int num_of_joints = joints_name_to_number_map_.size();
		state_msg_.desired.positions.resize(num_of_joints);
		state_msg_.desired.velocities.resize(num_of_joints);
		state_msg_.desired.accelerations.resize(num_of_joints);
		state_msg_.actual.positions.resize(num_of_joints);
		state_msg_.actual.velocities.resize(num_of_joints);
		state_msg_.error.positions.resize(num_of_joints);
		state_msg_.error.velocities.resize(num_of_joints);


		nh_.param<std::string>("follow_joint_trajectory_action_name", action_name_, "follow_joint_trajectory");
		as_ = new ActionServerType(nh_, action_name_,
				boost::bind(&TrajectoryExecuter::trajectoryActionCallback, this, _1), false);
		as_->start();
	}

	~TrajectoryExecuter() {
		delete as_;
	}

	void main() {
		while (true) {
			{
				boost::unique_lock<boost::mutex> lock(mut_);
				waiting_ = true;
				bool gotit = false;
				while (!run_){
					if(!nh_.ok()) {
						return;
					}
					try {
						boost::this_thread::interruption_point();
						gotit = cond_.timed_wait(lock, boost::posix_time::milliseconds(100));
					}
					catch(const boost::thread_interrupted&) {
						ROS_INFO("Trajectory thread was interrupted and returns, stopping the arm.");
						arm_->normalStopAll();
						return;
					}
					if (! gotit) {
						//Let's make people happy by publishing a state message
						for (unsigned int joint_i = 0; joint_i<joints_name_to_number_map_.size(); ++joint_i) {
							//fill in the message
							state_msg_.desired = state_msg_.actual;
							state_msg_.actual.positions[joint_i] = arm_->getPosition(joint_i);
							state_msg_.actual.velocities[joint_i] = 0;
							state_msg_.actual.time_from_start = ros::Duration(0);
						}

						//finally publish the state message
						state_msg_.header.stamp = ros::Time::now();
						state_publisher_.publish(state_msg_);
					}

				}
			}
			//I hope the mutex is unlocked now
			{
				boost::unique_lock<boost::mutex> lock(mut_);
				waiting_ = false;
			}

			ROS_INFO("Trajectory thread is starting its job");
			followTrajectory();
			arm_->normalStopAll();

			{
				//Self deactivate
				boost::unique_lock<boost::mutex> lock(mut_);
				run_ = false;
			}
			ROS_INFO("Trajectory thread has finished its job");
		}
	}

	void start(const trajectory_msgs::JointTrajectory& newtraj) {
		{
			boost::unique_lock<boost::mutex> lock(mut_);
			if (run_) {
				ROS_WARN("You need to stop a trajectory before issuing a new command. Ignoring the new command");
				return;
			}

			traj_ = newtraj;
			run_ = true;
		}
		ROS_INFO("Waking up the thread");
		cond_.notify_one();
	}

	void stop() {
		boost::unique_lock<boost::mutex> lock(mut_);
		run_ = false;
	}

	bool isRunning() {
		boost::unique_lock<boost::mutex> lock(mut_);
		return run_;
	}

	bool isWaiting() {
		boost::unique_lock<boost::mutex> lock(mut_);
		return waiting_;
	}


	// TODO: DOC choose this only if the velocity and time are calculated precisely
	void trajectoryActionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
	{
		const trajectory_msgs::JointTrajectory& traj = goal->trajectory;
		const unsigned int num_of_joints = traj.joint_names.size();

		// joint order does not change within trajectory, therefore
		//  to prevent superfluous joint name comparisons this array maps the
		//  index in the message  (usually joint_i) to the id in the model
		RobotArm::IDType msg_index_to_model_index[num_of_joints];
		// for each joint in step... map id
		for (unsigned int joint_i = 0; joint_i < num_of_joints; ++joint_i) {
			RobotArm::IDType id = joints_name_to_number_map_[traj.joint_names[joint_i]];
			msg_index_to_model_index[joint_i] = id;
			ROS_INFO_STREAM("TrajectoryAction callback: mapping joint: msg id "<<joint_i<<" to model id "<<id);
		}


		// check arm conditions before starting trajectory execution
		for (unsigned int joint_i = 0; joint_i < num_of_joints; ++joint_i) {
			RobotArm::IDType id = msg_index_to_model_index[joint_i];

			// check joint status
			uint8_t moving, error, brake, foo;
			float foofl;
			arm_->getModuleStatus(id, foo, moving, foo, foo, error, brake, foo, foo, foo, foofl);
			if(moving || error) {
				arm_->normalStopAll();
				ROS_ERROR("Arm in error state or moving, rejecting trajectory action.");
				as_->setAborted();
				return;
			}

			// check joint position
			float MAX_ALLOWED_DEVIANCE = 0.05; // rad
			float first_step_position = traj.points[0].positions[joint_i];
			float current_position = arm_->getPosition(id);
			float actual_difference = first_step_position-current_position;
			if (abs(actual_difference) > MAX_ALLOWED_DEVIANCE) {
				arm_->normalStopAll();
				ROS_ERROR_STREAM("Arm pose and first trajectory step differ to much, "
						<< "rejecting trajectory action. Joint ID: " << id <<
						" difference: " << actual_difference << " allowed: " << MAX_ALLOWED_DEVIANCE);
				as_->setAborted();
				return;
			}

		}
		ROS_INFO("Arm in valid state, starting trajectory..");


		// init feedback message
		feedback_.joint_names = traj.joint_names;
		feedback_.desired.positions.resize(num_of_joints);
		feedback_.desired.velocities.resize(num_of_joints);
		feedback_.desired.accelerations.resize(num_of_joints);
		feedback_.actual.positions.resize(num_of_joints);
		feedback_.actual.velocities.resize(num_of_joints);
		feedback_.actual.accelerations.resize(num_of_joints);
		feedback_.error.positions.resize(num_of_joints);
		feedback_.error.velocities.resize(num_of_joints);
		feedback_.error.accelerations.resize(num_of_joints);

		const ros::Time traj_start_time = ros::Time::now();

		////////////
		// for each trajectory step...
		unsigned int steps = traj.points.size();
		for (unsigned int step_i = 0; step_i < steps; ++step_i) {

			// check that preempt has not been requested by the client
			if (as_->isPreemptRequested() || !ros::ok()) {
				arm_->normalStopAll();
				ROS_INFO("%s: Preempted", action_name_.c_str());
				as_->setPreempted();
				return;
			}

			// check no module is in error state
			for (unsigned int joint_i = 0; joint_i < num_of_joints; ++joint_i) {
				RobotArm::IDType id = msg_index_to_model_index[joint_i];
				uint8_t error, foo;
				float foofl;
				arm_->getModuleStatus(id, foo, foo, foo, foo, error, foo, foo, foo, foo, foofl);
				if(error) {
					arm_->normalStopAll();
					ROS_ERROR("Arm in error state, aborting trajectory action.");
					as_->setAborted();
					return;
				}
			}

			// get the step target
			const trajectory_msgs::JointTrajectoryPoint& traj_step = traj.points[step_i];
			feedback_.desired = traj_step;

			ROS_INFO_STREAM("Trajectory thread executing step "<<step_i<<"+1/"<<steps<<" until "<<traj_step.time_from_start<<" s");

			////////////
			// for each joint in step... move
			for (unsigned int joint_i = 0; joint_i < num_of_joints; ++joint_i) {

				RobotArm::IDType id = msg_index_to_model_index[joint_i];
				float acceleration = traj_step.accelerations[joint_i];
				float velocity = traj_step.velocities[joint_i];
				float position = traj_step.positions[joint_i];

//				ROS_DEBUG_STREAM("Step for module "<<id<<": "<<setiosflags(ios::fixed)<<velocity<<" rad/s, "<<acceleration<<" rad/s*s, "<<position<<" rad");

				if (isnanf(acceleration) || isnanf(velocity) || isnanf(position)) {
					arm_->normalStopAll();
					ROS_ERROR("Received NAN in trajectory step! Aborting trajectory action.");
					ROS_ERROR_STREAM(traj);
					as_->setAborted();
					return;
				}

				if(step_i == steps-1) {
//					ROS_INFO("Last step, moving to position instead of with velocity.");
					velocity = 0.05;
					arm_->setVelocity(id, velocity);
					arm_->movePosition(id, position);
				}
				else if(abs(velocity) < TOO_SLOW) {
					// make joints near step goal reach that goal via position
					// as the velocity might be too slow already
					ROS_INFO_STREAM("Velocity for module "<<id<<" to close to zero to move with velocity, therefore moving to position with fix velocity.");
					velocity = 0.05;
					arm_->setVelocity(id, velocity);
					arm_->movePosition(id, position);
				}
				else {
					arm_->moveVelocity(id, velocity);
				}
			}

			// for each joint in step... feedback
			for (unsigned int joint_i = 0; joint_i < num_of_joints; ++joint_i) {
				// fill the joint individual feedback part
				RobotArm::IDType id = msg_index_to_model_index[joint_i];
				feedback_.actual.positions[joint_i] = arm_->getPosition(id);
				feedback_.actual.velocities[joint_i] = arm_->getVelocity(id);	// Note: not supported in SchunkProtocol
			}

			// publish the feedback
			feedback_.header.stamp = ros::Time::now();
			feedback_.actual.time_from_start = feedback_.header.stamp - traj_start_time;
			as_->publishFeedback(feedback_);

			// sleep until step time has passed
			ros::Time step_end_time = traj_start_time + traj_step.time_from_start;
			if(step_end_time <= ros::Time::now())
				ROS_WARN("Trajectory follower to slow, has to sleep a negative duration!");
			else
				ros::Time::sleepUntil(step_end_time);

			// TODO: what about a check if joints have moved beyond the goal and stop them?

//			// the arm is too slow, wait until each joint is near its goal
//	note: this does not work (unless fixed) when using moveVelocity, as the joint
//	could have passed the step goal, in which case waiting would be disastrous
//			ROS_INFO("Waiting for arm to reach trajectory step..");
//			bool all_joints_near_goal;
//			do {
//				ros::WallDuration(0.06).sleep();
//				all_joints_near_goal = true;
//				for (unsigned int joint_i = 0; joint_i < num_of_joints; ++joint_i) {
//					RobotArm::IDType id = msg_index_to_model_index[joint_i];
//
//					float position_goal = traj_step.positions[joint_i];
//					float position_current = arm_->getPosition(id);
//					float position_deviation = std::abs(position_goal - position_current);
//
//					if(position_deviation > 0.1) {
//						ROS_INFO_STREAM("Module "<<id<<" not yet near goal: deviation="<<position_deviation<<
//								" goal="<<position_goal<<" current="<<position_current<<" rad");
//						all_joints_near_goal = false;
//						break;
//					}
//				}
//
//				// check that preempt has not been requested by the client
//				if (as_->isPreemptRequested() || !ros::ok()) {
//					ROS_INFO("%s: Preempted", action_name_.c_str());
//					// set the action state to preempted
//					as_->setPreempted();
//					arm_->normalStopAll();
//					return;
//				}
//			} while(!all_joints_near_goal);

		}//for each trajectory step...


		ROS_INFO("Trajectory done, waiting for joints to stop...");
		bool all_joints_stopped;
		do {
			ros::WallDuration(0.06).sleep();
			all_joints_stopped = true;
			for (unsigned int joint_i = 0; joint_i < num_of_joints; ++joint_i) {
				RobotArm::IDType id = msg_index_to_model_index[joint_i];
				uint8_t moving, brake, foo;
				float foofl;
				arm_->getModuleStatus(id, foo, moving, foo, foo, foo, brake, foo, foo, foo, foofl);
				if(moving) {
					all_joints_stopped = false;
					break;
				}
			}
		} while (!all_joints_stopped);

		result_.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
		ROS_INFO("%s: Succeeded", action_name_.c_str());
		// set the action state to succeeded
		as_->setSucceeded(result_);
	}


	// from action tutorial
protected:
	ros::NodeHandle nh_;
	typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> ActionServerType;
	ActionServerType* as_;
	std::string action_name_;
	// create messages that are used to published feedback/result
	control_msgs::FollowJointTrajectoryFeedback feedback_;
	control_msgs::FollowJointTrajectoryResult result_;


private:
	/* At this time only velocity values are followed. */
	void followTrajectory() {
		//I am not entirely sure that this code actually looks like real-time
		ros::Time trajStartTime = ros::Time::now();
		for (unsigned int step = 0; step < traj_.points.size(); ++step) {
			trajectory_msgs::JointTrajectoryPoint& point = traj_.points[step];

			ROS_INFO_STREAM("Trajectory thread executing step "<<step);
			//now apply speed to each joint
			for (unsigned int joint_i = 0; joint_i < traj_.joint_names.size(); ++joint_i) {

				RobotArm::IDType id = joints_name_to_number_map_[traj_.joint_names[joint_i]];
				float velocity = point.velocities[joint_i];

				ROS_INFO_STREAM("Moving module "<<id<<" with "<<velocity<<" rad/s = "<<RAD_TO_DEG(velocity)<<" deg/s");
				arm_->moveVelocity(id, velocity);

				//fill in the message
				state_msg_.desired = state_msg_.actual;
				state_msg_.actual.positions[joint_i] = arm_->getPosition(joint_i);
				state_msg_.actual.velocities[joint_i] = arm_->getVelocity(joint_i);	// Note: not supported on SchunkProtocol
				state_msg_.actual.time_from_start = point.time_from_start;
			}

			//finally publish the state message
			state_msg_.header.stamp = ros::Time::now();
			state_publisher_.publish(state_msg_);


			//wait for the right time and check if it has to die
			while (ros::Time::now() < trajStartTime + point.time_from_start) {
				; // <- Now things are really dirty
				{ //This is tricky... a block as the only body of a loop!
					boost::unique_lock<boost::mutex> lock(mut_);
					if (!run_) {
						ROS_INFO("Trajectory thread has been commanded to stop");
						return;
					}
				}
			}
		}
	}

	RobotArm* arm_;
	std::map<std::string, RobotArm::IDType> joints_name_to_number_map_;
	trajectory_msgs::JointTrajectory traj_;
	bool run_;
	bool waiting_;
	boost::condition_variable cond_;
	boost::mutex mut_;
	control_msgs::JointTrajectoryControllerState state_msg_;
	ros::Publisher state_publisher_;
};



class SchunkServer :
		public AmtecManipulatorMod::ModuleStatusCallbackMod/*,
		private boost::noncopyable*/ {
public:
	SchunkServer(ros::NodeHandle &nh) :
		node_handle_(nh),
		arm_(NULL),
		trajectory_executer_(nh)
	{
		init();
	}

	~SchunkServer() {
		trajectory_executer_.stop();
		trajectory_executer_thread_.interrupt();
		trajectory_executer_thread_.join();
		if(arm_ != NULL)
			delete arm_;
	}


	virtual void moduleStatusUpdate(uint8_t id, float pos, float vel, float current,
			AmtecManipulatorMod::StatusBits status, MTime last_update, bool is_running) {
//		ROS_INFO("moduleStatusUpdate #%d % 2f % 2f %2f %X", id, pos, vel, current, status.value);
		static MetraLabs::base::MTimeSpan joints_timeout(300);
		static MetraLabs::base::MTimeSpan last_status_age;

		MetraLabs::base::MTimeSpan status_age = MetraLabs::base::MTime::now() - last_update;
		if(status_age > joints_timeout) {
			ROS_WARN_STREAM("Joints status update too old! " << status_age.getTimeSpan()
					<< " ms (thread is running: " << is_running << ")"); // Quitting..
//			ROS_ERROR_STREM("Joints status was already ")
//			ros::shutdown(); // FATAL does not cleanly shutdown the robot
		}
//		else
//			ROS_INFO_STREAM("Joints status update age: " << status_age.getTimeSpan() << " ms");

		if(!is_running) {
			ROS_ERROR("Joints update thread is not running! Quitting..");
			ros::shutdown(); // FATAL does not cleanly shutdown the robot
		}
	}

private:
	void renewRobotArm() {
		// delete old instance if existent
		if(arm_ != NULL)
			delete arm_;

		std::string robot_arm_class;
		ros::NodeHandle nh_private("~");
		if(!nh_private.getParam("robot_arm_class", robot_arm_class)) {
			ROS_WARN_STREAM("Parameter robot arm class not set, defaulting to AmtecProtocolArm.");
			robot_arm_class = "AmtecProtocolArm";
		}

		// TODO implement real class loader
		if(robot_arm_class == "AmtecProtocolArm")
			arm_ = new AmtecProtocolArm();
//		else if(robot_arm_class == "SchunkProtocolArm")
//			arm_ = new SchunkProtocolArm();
		else if(robot_arm_class == "LWA3ArmUASHH")
			arm_ = new LWA3ArmUASHH();
//		else if(robot_arm_class == "PowerCubeArmUUISRC")
//			arm_ = new PowerCubeArmUUISRC();
		else {
			ROS_FATAL_STREAM("Cannot load unknown robot arm class: "<<robot_arm_class);
			exit(1);
		}
	}

	void init() {
		// Initialise the arm model parser and find out what non-fixed joints are present
		arm_model_.initParam("robot_description");
		std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator mapElement;
		for (mapElement = arm_model_.joints_.begin(); mapElement != arm_model_.joints_.end(); ++mapElement) {
			boost::shared_ptr<urdf::Joint> joint = mapElement->second;
			ROS_DEBUG_STREAM("Joint: Name="<<joint->name<<" MimicPtr="<<joint->mimic);
			if (joint->type != urdf::Joint::FIXED &&
					joint->mimic == 0)	// ignore virtual joints mimicking a physical one
				joints_list_.push_back(joint);
		}
		ROS_INFO("URDF specifies %d non-fixed non-mimicking joints.", joints_list_.size());

		// Initialise the arm
		renewRobotArm();
		arm_->init();

		// Check if reinitialisation is needed
		while(arm_->getModulesCount() != joints_list_.size()) {
			ROS_WARN("Reinitialising robot arm... found %d joints but need %d", arm_->getModulesCount(), joints_list_.size());
			renewRobotArm();
			ros::WallDuration(1).sleep(); // let arm start up
			arm_->init();
		}

		// Check that the required modules are present.
		ROS_WARN("Didn't check that the modules in the description match modules in reality.");

		// Set up the joint state publisher with the joint names
		// This assumes that the joints are ordered on the robot in the same order as the URDF!!!
		current_JointState_.name.resize(joints_list_.size());
		current_JointState_.position.resize(joints_list_.size());
		current_JointState_.velocity.resize(joints_list_.size());
		for (unsigned int i = 0; i < joints_list_.size(); ++i) {
			current_JointState_.name[i] = joints_list_[i]->name;
			joints_name_to_number_map_[joints_list_[i]->name] = i;
			ROS_INFO("%d is mapping to %s", i, joints_list_[i]->name.c_str());
		}
		current_JointState_publisher_ = node_handle_.advertise<sensor_msgs::JointState>("/joint_states", 1);

		// Set up the schunk status publisher
		current_SchunkStatus_publisher_ = node_handle_.advertise<metralabs_msgs::SchunkStatus>("status", 1);
		for (uint i = 0; i < joints_list_.size(); ++i) {
			metralabs_msgs::SchunkJointStatus status;
			status.jointName = joints_list_[i]->name;
			current_SchunkStatus_.joints.push_back(status);
		}

		trajectory_executer_.init(arm_, joints_name_to_number_map_);

		ROS_INFO("Starting the trajectory executer thread");
		trajectory_executer_thread_ = boost::thread(boost::bind(&TrajectoryExecuter::main, &trajectory_executer_));


		/*
		 * /schunk/position/joint_state -> publishes joint state for kinematics modules
		 * /schunk/target_pc/joint_state <- for received position control commands
		 * /schunk/target_vc/joint_state  <- for receiving velocity control commands
		 * /schunk/status -> to publish all the statuses
		 */

		ROS_INFO("Subscribing schunk topics...");

		// those topics which must be received multiple times (for each joint) got a 10 for their message buffer
		emergency_subscriber_ = node_handle_.subscribe("emergency", 1, &SchunkServer::cb_emergency, this);
		stop_subscriber_ = node_handle_.subscribe("stop", 1, &SchunkServer::cb_stop, this);
		ack_subscriber_ = node_handle_.subscribe("ack", 10, &SchunkServer::cb_ack, this);
		ack_all_subscriber_ = node_handle_.subscribe("ack_all", 1, &SchunkServer::cb_ackAll, this);
		ref_subscriber_ = node_handle_.subscribe("ref", 10, &SchunkServer::cb_ref, this);
		ref_all_subscriber_ = node_handle_.subscribe("ref_all", 1, &SchunkServer::cb_refAll, this);

		set_velocity_subscriber_ = node_handle_.subscribe("set_velocity", 10, &SchunkServer::cb_setVelocity, this);
		set_acceleration_subscriber_ = node_handle_.subscribe("set_acceleration", 10, &SchunkServer::cb_setAcceleration, this);
		set_current_subscriber_ = node_handle_.subscribe("set_current", 10, &SchunkServer::cb_setCurrent, this);
		set_currents_max_all_subscriber_ = node_handle_.subscribe("set_current_max_all", 1, &SchunkServer::cb_setCurrentsMaxAll, this);

		move_position_subscriber_ = node_handle_.subscribe("move_position", 10, &SchunkServer::cb_movePosition, this);
		move_velocity_subscriber_ = node_handle_.subscribe("move_velocity", 10, &SchunkServer::cb_moveVelocity, this);
		move_all_position_subscriber_ = node_handle_.subscribe("move_all_position", 1, &SchunkServer::cb_moveAllPosition, this);
		move_all_velocity_subscriber_ = node_handle_.subscribe("move_all_velocity", 1, &SchunkServer::cb_moveAllVelocity, this);
		trajectory_command_subscriber_ = node_handle_.subscribe("trajectory_command", 1, &SchunkServer::cb_commandTrajectory, this);

		boost::thread(&SchunkServer::publishingLoop, this, ros::Rate(30));

		AmtecProtocolArm* ap_arm = static_cast<AmtecProtocolArm*>(arm_);
		ap_arm->getManipulator().addModuleStatusCallback(this, 0);

		ROS_INFO("SchunkServer Ready");

	}

	void cb_emergency(const std_msgs::Empty::ConstPtr& dummy) 	{
		ROS_INFO("emergency");
		arm_->emergencyStopAll();
	}

	void cb_stop(const std_msgs::Empty::ConstPtr& dummy) 	{
		ROS_INFO("stop");
		arm_->normalStopAll();
	}

	void cb_ack(const std_msgs::Int8::ConstPtr& id) 	{
		ROS_INFO("cb_ack: [%d]", id->data);
		arm_->ackJoint(id->data);
	}

	void cb_ackAll(const std_msgs::Empty::ConstPtr& dummy) 	{
		ROS_INFO("cb_ackAll");
		arm_->ackAll();
	}

	void cb_ref(const std_msgs::Int8::ConstPtr& id)	{
		ROS_INFO("cb_ref: [%d]", id->data);
		arm_->refJoint(id->data);
	}

	void cb_refAll(const std_msgs::Empty::ConstPtr& dummy)	{
		ROS_INFO("cb_refAll");
		arm_->refAll();
	}


	void cb_setVelocity(const metralabs_msgs::IDAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_setVelocity: [%d, %f]", data->id, data->value);
		arm_->setVelocity(data->id, data->value);
	}

	void cb_setAcceleration(const metralabs_msgs::IDAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_setAcceleration: [%d, %f]", data->id, data->value);
		arm_->setAcceleration(data->id, data->value);
	}

	void cb_setCurrent(const metralabs_msgs::IDAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_setCurrent: [%d, %f]", data->id, data->value);
		arm_->setCurrent(data->id, data->value);
	}

	void cb_setCurrentsMaxAll(const std_msgs::Empty::ConstPtr& dummy)	{
		ROS_INFO("cb_setCurrentsMaxAll");
		arm_->setCurrentsToMax();
	}


	void cb_movePosition(const metralabs_msgs::IDAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_movePosition: [%d, %f]", data->id, data->value);
		arm_->movePosition(data->id, data->value);
	}

	void cb_moveVelocity(const metralabs_msgs::IDAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_moveVelocity: [%d, %f]", data->id, data->value);
		if (data->value == 0.0)
			arm_->normalStopJoint(data->id);
		else
			arm_->moveVelocity(data->id, data->value);
	}

	void cb_moveAllPosition(const sensor_msgs::JointState::ConstPtr& data) {
		// The "names" member says how many joints, the other members may be empty.
		// Not all the joints have to be specified in the message
		// and not all types must be filled
		for (uint i = 0; i < data.get()->name.size(); ++i) {
			string joint_name = data.get()->name[i];
			RobotArm::IDType joint_number = joints_name_to_number_map_[joint_name];
			ROS_INFO_STREAM("cb_PositionControl: joint "<<joint_name<<" (#"<<joint_number<<")");

			arm_->setCurrentsToMax();
			if (data.get()->position.size()!=0) {
				double pos = data.get()->position[i];
				ROS_INFO_STREAM(" to position "<<pos);
				arm_->movePosition(joint_number, pos);
			}
			if (data.get()->velocity.size()!=0) {
				double vel = data.get()->velocity[i];
				ROS_INFO_STREAM(" with velocity "<<vel);
				arm_->setVelocity(joint_number, vel);
			}
			if (data.get()->effort.size()!=0) {
				double eff = data.get()->effort[i];
				ROS_INFO_STREAM(" with effort "<<eff);
				arm_->setCurrent(joint_number, eff);
			}
		}
	}

	void cb_moveAllVelocity(const sensor_msgs::JointState::ConstPtr& data) {
		// The "names" member says how many joints, the other members may be empty.
		// Not all the joints have to be specified in the message
		// and not all types must be filled
		for (uint i = 0; i < data.get()->name.size(); ++i) {
			string joint_name = data.get()->name[i];
			RobotArm::IDType joint_number = joints_name_to_number_map_[joint_name];
			ROS_INFO_STREAM("cb_VelocityControl: joint "<<joint_name<<" (#"<<joint_number<<")");

//			m_powerCube.setCurrentsToMax();

			if (data.get()->velocity.size() != 0) {
				double vel = data.get()->velocity[i];
				ROS_INFO_STREAM(" with velocity "<<vel);
				if (vel == 0.0)
					arm_->normalStopJoint(joint_number);
				else
					arm_->moveVelocity(joint_number, vel);
			}
			if (data.get()->effort.size()!=0) {
				double eff = data.get()->effort[i];
				ROS_INFO_STREAM(" with effort "<<eff);
				arm_->setCurrent(joint_number, eff);
			}
		}
	}

	void cb_commandTrajectory(const trajectory_msgs::JointTrajectory traj) {
		ROS_INFO("Schunk Server: received a new trajectory");
		trajectory_executer_.stop();
		while (! trajectory_executer_.isWaiting() ) {
			ROS_INFO("Waiting for the controller to be ready..");
		}
		trajectory_executer_.start(traj);
	}

private:


	void publishingLoop(ros::Rate loop_rate) {
		// TODO: convert to publisher
//		ros::Duration(10).sleep();
		ROS_WARN("(TEST) from now on I'm publishing joint states!");
		while (node_handle_.ok()) {
			publishCurrentJointState();
			publishCurrentSchunkStatus();
			// This will adjust as needed per iteration
			loop_rate.sleep();
		}
	}

	void publishCurrentJointState() {
		current_JointState_.header.stamp = ros::Time::now();
		for (unsigned int i = 0; i < current_JointState_.name.size(); ++i) {
			current_JointState_.position[i] = arm_->getPosition(i);
			current_JointState_.velocity[i] = arm_->getVelocity(i);
		}
		current_JointState_publisher_.publish(current_JointState_);
	}

	void publishCurrentSchunkStatus() {
		std::vector<metralabs_msgs::SchunkJointStatus>::iterator it;
		for (it = current_SchunkStatus_.joints.begin(); it != current_SchunkStatus_.joints.end(); ++it) {
			RobotArm::IDType module_number = joints_name_to_number_map_[it->jointName];
			arm_->getModuleStatus(module_number, it->referenced, it->moving, it->progMode, it->warning,
					it->error, it->brake, it->moveEnd, it->posReached, it->errorCode, it->current);
		}
		current_SchunkStatus_publisher_.publish(current_SchunkStatus_);
	}


private:
	ros::NodeHandle node_handle_;
	RobotArm* arm_;
	urdf::Model arm_model_;	// A parsing of the model description
	std::vector<boost::shared_ptr<urdf::Joint> > joints_list_;
	std::map<std::string, RobotArm::IDType> joints_name_to_number_map_;

	sensor_msgs::JointState current_JointState_;
	metralabs_msgs::SchunkStatus current_SchunkStatus_;
	ros::Publisher current_JointState_publisher_;
	ros::Publisher current_SchunkStatus_publisher_;

	TrajectoryExecuter trajectory_executer_;
	boost::thread trajectory_executer_thread_;

	ros::Subscriber emergency_subscriber_;
	ros::Subscriber stop_subscriber_;
	ros::Subscriber first_ref_subscriber_;
	ros::Subscriber ack_subscriber_;
	ros::Subscriber ack_all_subscriber_;
	ros::Subscriber ref_subscriber_;
	ros::Subscriber ref_all_subscriber_;

	ros::Subscriber set_velocity_subscriber_;
	ros::Subscriber set_acceleration_subscriber_;
	ros::Subscriber set_current_subscriber_;
	ros::Subscriber set_currents_max_all_subscriber_;

	ros::Subscriber move_position_subscriber_;
	ros::Subscriber move_velocity_subscriber_;
	ros::Subscriber move_all_position_subscriber_;
	ros::Subscriber move_all_velocity_subscriber_;
	ros::Subscriber trajectory_command_subscriber_;
};


#endif /* SCHUNKSERVER_H_ */
