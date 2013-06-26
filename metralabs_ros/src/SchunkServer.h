#ifndef SCHUNKSERVER_H_
#define SCHUNKSERVER_H_

#include <boost/thread.hpp>
#include <boost/noncopyable.hpp>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <urdf/model.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

#include <metralabs_ros/idAndFloat.h>
#include <metralabs_ros/SchunkStatus.h>

#include <sensor_msgs/JointState.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include "PowerCube.h"


#define DEG_TO_RAD(d)	((d)*M_PI/180.0)
#define RAD_TO_DEG(r)	((r)*180.0/M_PI)

#define SLOWEST_REASONABLE_JOINT_SPEED	0.015f // the slowest reasonable speed for our joints (IMHO)
#define TOO_SLOW	0.002f //



class TrajectoryExecuter : private boost::noncopyable {
public:
	TrajectoryExecuter(ros::NodeHandle& nh_, std::string action_server_name) :
		as_(nh_, action_server_name, boost::bind(&TrajectoryExecuter::trajectoryActionCallback, this, _1), false),
		action_name_(action_server_name)
	{
		run_ = false;
		waiting_ = false;
		arm_ = NULL;
	}

	void init(ros::NodeHandle& n, PowerCube* p_arm, std::map<std::string, unsigned int>& nmap) {

		arm_ = p_arm;
		joints_name_to_number_map_ = nmap;

		state_publisher_ = n.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>("state", 1);

		std::map<std::string, unsigned int>::iterator it;
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

		as_.start();
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
							state_msg_.actual.positions[joint_i] = RAD_TO_DEG(arm_->getManipulator().getModules().at(joint_i).status_pos);
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
		return; //just to be sure the thread is terminated
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


	void trajectoryActionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
	{
		bool success = true;
		ros::Time trajStartTime = ros::Time::now();
		const trajectory_msgs::JointTrajectory& traj = goal->trajectory;

		// init feedback message
		feedback_.joint_names = traj.joint_names;
		int num_of_joints = traj.joint_names.size();
		feedback_.desired.positions.resize(num_of_joints);
		feedback_.desired.velocities.resize(num_of_joints);
		feedback_.desired.accelerations.resize(num_of_joints);
		feedback_.actual.positions.resize(num_of_joints);
		feedback_.actual.velocities.resize(num_of_joints);
		feedback_.actual.accelerations.resize(num_of_joints);
		feedback_.error.positions.resize(num_of_joints);
		feedback_.error.velocities.resize(num_of_joints);
		feedback_.error.accelerations.resize(num_of_joints);

		// for each trajectory step...
		unsigned int steps = traj.points.size();
		for (unsigned int step_i = 0; step_i < steps; ++step_i) {

			// check that preempt has not been requested by the client
			if (as_.isPreemptRequested() || !ros::ok()) {
				ROS_INFO("%s: Preempted", action_name_.c_str());
				// set the action state to preempted
				as_.setPreempted();
				success = false;
				arm_->normalStopAll();
				break;
			}


			// get the step target
			const trajectory_msgs::JointTrajectoryPoint& trajStep = traj.points[step_i];
			feedback_.desired = trajStep;

			ROS_INFO_STREAM("Trajectory thread executing step "<<step_i<<"+1/"<<steps<<" until "<<trajStep.time_from_start<<" s / "<<(trajStartTime+trajStep.time_from_start));

			// for each joint in step...
			for (unsigned int joint_i = 0; joint_i < traj.joint_names.size(); ++joint_i) {

				int id = joints_name_to_number_map_[traj.joint_names[joint_i]];

				float accRad = trajStep.accelerations[joint_i];
				float velRad = trajStep.velocities[joint_i];
				float posRad = trajStep.positions[joint_i];

//				ROS_INFO_STREAM("Moving module "<<id<<" with "<</*setiosflags(ios::fixed)<<*/ velRad<<" rad/s and "<<accRad<<" rad/s*s to "<<posRad<<" rad");


/// don't go below +- def
#define SIGN_TOLERANT_MAX(value, def)	( (value)==0 ? 0 : \
											( \
												(value)>0 ? \
													std::max((value),  (def)) : \
													std::min((value), -(def)) \
											) \
										)
/// don't ...
#define SIGN_TOLERANT_CUT(value, lim)	( (value)==0 ? 0 : \
											( \
												(value)>0 ? \
													( (value) >  (lim) ? (value) : (0) ) : \
													( (value) < -(lim) ? (value) : (0) ) \
											) \
										)

//				velRad = SIGN_TOLERANT_CUT(velRad, TOO_SLOW);
//				velRad = SIGN_TOLERANT_MAX(velRad, SLOWEST_REASONABLE_JOINT_SPEED);
//				accRad = SIGN_TOLERANT_MAX(accRad, 0.43f);
				accRad = 0.21;

				if(step_i == steps-1) {
					velRad = SLOWEST_REASONABLE_JOINT_SPEED;
				}

//				if(std::abs(velRad) < TOO_SLOW) {
//					ROS_INFO_STREAM("Skipping module "<<id<<" due to too low velocity");
//				}
//				else {
				ROS_INFO_STREAM("Moving module "<<id<<" with "<</*setiosflags(ios::fixed)<<*/ velRad<<" rad/s and "<<accRad<<" rad/s*s to "<<posRad<<" rad");

#if SCHUNK_NOT_AMTEC != 0
				// TODO schunk option of trajectory action callback
//				arm_->moveVelocity(id, velDeg);
#else
				arm_->setTargetAcceleration(id, accRad);
				arm_->setTargetVelocity(id, velRad);
				arm_->movePosition(id, posRad);
//				arm_->moveVelocity(id, velRad);	TODO choose this if the velocity and time are calculated precisely

//				if(step_i == steps-1) {
//					ROS_INFO("LAST STEP, moving to position instead of with velocity.");
//					arm_->movePosition(id, posRad);
//				}
//				else {
////					ROS_INFO("Velocity move.");
//					arm_->moveVelocity(id, velRad);
//				}
#endif
//				}

				// fill the joint individual feedback part
				feedback_.actual.positions[id] = arm_->getManipulator().getModules().at(id).status_pos;
#if SCHUNK_NOT_AMTEC != 0
				// no status_vel in schunk protocol
#else
				feedback_.actual.velocities[id] = arm_->getManipulator().getModules().at(id).status_vel;
#endif
//				feedback_.actual.accelerations[id] = arm_->manipulator_.getModules().at(id).status_acc / max_acceleration?; TODO cannot know acceleration status
			}

			// publish the feedback
			feedback_.header.stamp = ros::Time::now();
			feedback_.actual.time_from_start = ros::Time::now() - trajStartTime;
			as_.publishFeedback(feedback_);

			// sleep until step time has passed
			ros::Time::sleepUntil(trajStartTime + trajStep.time_from_start);
		}//for each trajectory step...

		if(success) {
			ROS_INFO("Trajectory done, waiting for joints to stop...");
			bool all_joints_stopped;
			do {
				ros::WallDuration(0.06).sleep();
				all_joints_stopped = true;
				for (unsigned int joint_i = 0; joint_i < traj.joint_names.size(); ++joint_i) {
					uint8_t moving, brake, foo;
					float foofl;
					arm_->getModuleStatus(joint_i, foo, moving, foo, foo, foo, brake, foo, foo, foo, foofl);
//					if(moving) {
					if(!brake) {
						all_joints_stopped = false;
						break;
					}
//					if(arm_->manipulator_.getModules().at(joint_i).status_vel == 0)
//						all_joints_stopped = false;
				}
			} while (!all_joints_stopped);

			result_.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
			ROS_INFO("%s: Succeeded", action_name_.c_str());
			// set the action state to succeeded
			as_.setSucceeded(result_);
		}
	}


	// from action tutorial
protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
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

				unsigned int id = joints_name_to_number_map_[traj_.joint_names[joint_i]];
				float velRad = point.velocities[joint_i];
				float velDeg = RAD_TO_DEG(velRad);

				ROS_INFO_STREAM("Moving module "<<id<<" with "<<velRad<<" rad/s = "<<velDeg<<" deg/s");
#if SCHUNK_NOT_AMTEC != 0
				arm_->moveVelocity(id, velDeg);
#else
				arm_->moveVelocity(id, velRad); // amtec protocol already in rad
#endif

				//fill in the message
				state_msg_.desired = state_msg_.actual;
#if SCHUNK_NOT_AMTEC != 0
				state_msg_.actual.positions[joint_i] = DEG_TO_RAD(arm_->getManipulator().getModules().at(joint_i).status_pos);
#else
				state_msg_.actual.positions[joint_i] = arm_->getManipulator().getModules().at(joint_i).status_pos;
#endif
				state_msg_.actual.velocities[joint_i] = point.velocities[joint_i];
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

	PowerCube* arm_;
	std::map<std::string, unsigned int> joints_name_to_number_map_;
	trajectory_msgs::JointTrajectory traj_;
	bool run_;
	bool waiting_;
	boost::condition_variable cond_;
	boost::mutex mut_;
	pr2_controllers_msgs::JointTrajectoryControllerState state_msg_;
	ros::Publisher state_publisher_;
};



class SchunkServer : private boost::noncopyable {
public:
	SchunkServer(ros::NodeHandle &node, std::string action_server_name) :
		node_handle_(node),
		trajectory_executer_(node, action_server_name)
	{
		init();
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

		// Initialise the powercube
		power_cube_.init();

		// Check if reinitialisation is needed
		while(power_cube_.getModulesCount() != joints_list_.size()) {
			ROS_WARN("Reinitialising robot arm... found %d joints but need %d", power_cube_.getModulesCount(), joints_list_.size());
			power_cube_.~PowerCube();
			ros::WallDuration(1).sleep(); // let arm start up
			new (&power_cube_) PowerCube;
			power_cube_.init();
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
		current_JointState_publisher_ = node_handle_.advertise<sensor_msgs::JointState>("/schunk/position/joint_states", 1);

		// Set up the schunk status publisher
		current_SchunkStatus_publisher_ = node_handle_.advertise<metralabs_ros::SchunkStatus>("/schunk/status", 1);
		for (uint i = 0; i < joints_list_.size(); ++i) {
			metralabs_ros::SchunkJointStatus status;
			status.jointName = joints_list_[i]->name;
			current_SchunkStatus_.joints.push_back(status);
		}

		trajectory_executer_.init(node_handle_, &power_cube_, joints_name_to_number_map_);

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
		emergency_subscriber_ = node_handle_.subscribe("/emergency", 1, &SchunkServer::cb_emergency, this);
		stop_subscriber_ = node_handle_.subscribe("/stop", 1, &SchunkServer::cb_stop, this);
		first_ref_subscriber_ = node_handle_.subscribe("/firstRef", 1, &SchunkServer::cb_firstRef, this);
		ack_subscriber_ = node_handle_.subscribe("/ack", 10, &SchunkServer::cb_ack, this);
		ack_all_subscriber_ = node_handle_.subscribe("/ackAll", 1, &SchunkServer::cb_ackAll, this);
		ref_subscriber_ = node_handle_.subscribe("/ref", 10, &SchunkServer::cb_ref, this);
		ref_all_subscriber_ = node_handle_.subscribe("/refAll", 1, &SchunkServer::cb_refAll, this);
		target_current_subscriber_ = node_handle_.subscribe("/current", 10, &SchunkServer::cb_targetCurrent, this);
		target_currents_max_all_subscriber_ = node_handle_.subscribe("/currentsMaxAll", 1, &SchunkServer::cb_targetCurrentsMaxAll, this);
		move_position_subscriber_ = node_handle_.subscribe("/movePosition", 10, &SchunkServer::cb_movePosition, this);
		move_velocity_subscriber_ = node_handle_.subscribe("/moveVelocity", 10, &SchunkServer::cb_moveVelocity, this);
		target_velocity_subscriber_ = node_handle_.subscribe("/targetVelocity", 10, &SchunkServer::cb_targetVelocity, this);
		target_acceleration_subscriber_ = node_handle_.subscribe("/targetAcceleration", 10, &SchunkServer::cb_targetAcceleration, this);

		move_all_position_subscriber_ = node_handle_.subscribe("/schunk/target_pc/joint_states", 1, &SchunkServer::cb_moveAllPosition, this);
		move_all_velocity_subscriber_ = node_handle_.subscribe("/schunk/target_vc/joint_states", 1, &SchunkServer::cb_moveAllVelocity, this);
		command_subscriber_ = node_handle_.subscribe("command", 1, &SchunkServer::cb_commandTrajectory, this);

		boost::thread(&SchunkServer::publishingLoop, this, ros::Rate(30));

		ROS_INFO("SchunkServer Ready");

	}

	~SchunkServer() {
		trajectory_executer_.stop();
		trajectory_executer_thread_.interrupt();
		trajectory_executer_thread_.join();
	}

	void cb_emergency(const std_msgs::Bool::ConstPtr& dummy) 	{
		ROS_INFO("emergency");
		power_cube_.emergencyStop();
	}

	void cb_stop(const std_msgs::Bool::ConstPtr& dummy) 	{
		ROS_INFO("stop");
		power_cube_.normalStopAll();
	}

	void cb_firstRef(const std_msgs::Bool::ConstPtr& dummy) 	{
		ROS_INFO("first ref");
		power_cube_.firstRef();
	}

	void cb_ack(const std_msgs::Int8::ConstPtr& id) 	{
		ROS_INFO("cb_ack: [%d]", id->data);
		power_cube_.ack(id->data);
	}

	void cb_ackAll(const std_msgs::Bool::ConstPtr& dummy) 	{
		ROS_INFO("cb_ackall");
		power_cube_.ackAll();
	}

	void cb_ref(const std_msgs::Int8::ConstPtr& id)	{
		ROS_INFO("cb_ref: [%d]", id->data);
		power_cube_.ref(id->data);
	}

	void cb_refAll(const std_msgs::Bool::ConstPtr& dummy)	{
		ROS_INFO("cb_refall");
		power_cube_.refAll();
	}

	void cb_targetCurrent(const metralabs_ros::idAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_current: [%d, %f]", data->id, data->value);
		power_cube_.setTargetCurrent(data->id, data->value);
	}

	void cb_targetCurrentsMaxAll(const std_msgs::Bool::ConstPtr& dummy)	{
		ROS_INFO("cb_currentsMax");
		power_cube_.setCurrentsToMax();
	}

	void cb_movePosition(const metralabs_ros::idAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_movePosition: [%d, %f]", data->id, data->value);
		power_cube_.movePosition(data->id, data->value);
	}

	void cb_moveVelocity(const metralabs_ros::idAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_moveVelocity: [%d, %f]", data->id, data->value);
		if (data->value == 0.0)
			power_cube_.normalStop(data->id);
		else
			power_cube_.moveVelocity(data->id, data->value);
	}

	void cb_targetVelocity(const metralabs_ros::idAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_targetVelocity: [%d, %f]", data->id, data->value);
		power_cube_.setTargetVelocity(data->id, data->value);
	}

	void cb_targetAcceleration(const metralabs_ros::idAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_targetAcceleration: [%d, %f]", data->id, data->value);
		power_cube_.setTargetAcceleration(data->id, data->value);
	}

	void cb_moveAllPosition(const sensor_msgs::JointState::ConstPtr& data) {
		// The "names" member says how many joints, the other members may be empty.
		// Not all the joints have to be specified in the message
		// and not all types must be filled
#if SCHUNK_NOT_AMTEC != 0
		float rad_to_degrees_needed = RAD_TO_DEG(1);
#else
		float rad_to_degrees_needed = 1;
#endif

		for (uint i = 0; i < data.get()->name.size(); ++i) {
			string joint_name = data.get()->name[i];
			int joint_number = joints_name_to_number_map_[joint_name];
			ROS_INFO_STREAM("cb_PositionControl: joint "<<joint_name<<" (#"<<joint_number<<")");

			power_cube_.setCurrentsToMax();
			if (data.get()->position.size()!=0) {
				double pos = data.get()->position[i] / rad_to_degrees_needed;
				ROS_INFO_STREAM(" to position "<<pos);
				power_cube_.movePosition(joint_number, pos);
			}
			if (data.get()->velocity.size()!=0) {
				double vel = data.get()->velocity[i] / rad_to_degrees_needed;
				ROS_INFO_STREAM(" with velocity "<<vel);
				power_cube_.setTargetVelocity(joint_number, vel);
			}
			if (data.get()->effort.size()!=0) {
				double eff = data.get()->effort[i];
				ROS_INFO_STREAM(" with effort "<<eff);
				power_cube_.setTargetCurrent(joint_number, eff);
			}

		}
	}

	void cb_moveAllVelocity(const sensor_msgs::JointState::ConstPtr& data) {
#if SCHUNK_NOT_AMTEC != 0
		float rad_to_degrees_needed = RAD_TO_DEG(1);
#else
		float rad_to_degrees_needed = 1;
#endif

		// The "names" member says how many joints, the other members may be empty.
		// Not all the joints have to be specified in the message
		// and not all types must be filled
//		m_powerCube.ackAll();
		for (uint i = 0; i < data.get()->name.size(); ++i) {
			string joint_name = data.get()->name[i];
			int joint_number = joints_name_to_number_map_[joint_name];
			ROS_INFO_STREAM("cb_VelocityControl: joint "<<joint_name<<" (#"<<joint_number<<")");

//			m_powerCube.setCurrentsToMax();

			if (data.get()->velocity.size() != 0) {
				double vel = data.get()->velocity[i] / rad_to_degrees_needed;
				ROS_INFO_STREAM(" with velocity "<<vel);
				if (vel == 0.0)
					power_cube_.normalStop(joint_number);
				else
					power_cube_.moveVelocity(joint_number, vel);
			}
			if (data.get()->effort.size()!=0) {
				double eff = data.get()->effort[i];
				ROS_INFO_STREAM(" with effort "<<eff);
				power_cube_.setTargetCurrent(joint_number, eff);
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
#if SCHUNK_NOT_AMTEC != 0
			SCHUNKMotionManipulator::ModuleConfig modCfg = power_cube_.getManipulator().getModules().at(i);
			current_JointState_.position[i]=DEG_TO_RAD(modCfg.status_pos);
			// no status_vel in schunk protocol
#else
			AmtecManipulator::ModuleConfig modCfg = power_cube_.getManipulator().getModules().at(i);
			current_JointState_.position[i]=modCfg.status_pos; // amtec protocol already in rad
			current_JointState_.velocity[i]=modCfg.status_vel;
#endif
		}
		current_JointState_publisher_.publish(current_JointState_);
	}

	void publishCurrentSchunkStatus() {
		std::vector<metralabs_ros::SchunkJointStatus>::iterator it;
		uint moduleNumber;
		for (it = current_SchunkStatus_.joints.begin(); it != current_SchunkStatus_.joints.end(); ++it) {
			moduleNumber = joints_name_to_number_map_[it->jointName];
			power_cube_.getModuleStatus(moduleNumber, it->referenced, it->moving, it->progMode, it->warning,
					it->error, it->brake, it->moveEnd, it->posReached, it->errorCode, it->current);
		}
		current_SchunkStatus_publisher_.publish(current_SchunkStatus_);
	}


private:
	ros::NodeHandle node_handle_;
	PowerCube power_cube_;
	urdf::Model arm_model_;	// A parsing of the model description
	std::vector<boost::shared_ptr<urdf::Joint> > joints_list_;
	std::map<std::string, unsigned int> joints_name_to_number_map_;

	sensor_msgs::JointState current_JointState_;
	metralabs_ros::SchunkStatus current_SchunkStatus_;
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
	ros::Subscriber target_current_subscriber_;
	ros::Subscriber target_currents_max_all_subscriber_;
	ros::Subscriber move_position_subscriber_;
	ros::Subscriber move_velocity_subscriber_;
	ros::Subscriber target_velocity_subscriber_;
	ros::Subscriber target_acceleration_subscriber_;

	ros::Subscriber move_all_position_subscriber_;
	ros::Subscriber move_all_velocity_subscriber_;
	ros::Subscriber command_subscriber_;
};


#endif /* SCHUNKSERVER_H_ */
