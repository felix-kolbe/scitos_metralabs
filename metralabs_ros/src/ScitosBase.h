#ifndef __SCITOSBASE__
#define __SCITOSBASE__

#include <boost/thread.hpp>
#include <boost/noncopyable.hpp>

#include <MetraLabsBase.h>
#include <config/MLRobotic_config.h>
#include <base/Application.h>
#include <robot/Robot.h>
#include <robot/RangeData.h>

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <std_msgs/Empty.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>

#include <metralabs_ros/ScitosG5Bumper.h>

#include <dynamic_reconfigure/server.h>
#include <metralabs_ros/ScitosG5Config.h>


using namespace MetraLabs::base;
using namespace MetraLabs::robotic::base;
using namespace MetraLabs::robotic::robot;


#define FEATURE_SONAR	( "SonarsActive" )


class ScitosBase : public BlackboardDataUpdateCallback, private boost::noncopyable {

public:
	ScitosBase(const char*, int pArgc, char* pArgv[], ros::NodeHandle& nh);
	~ScitosBase();

	virtual void dataChanged(const BlackboardData* pData) {
		if(pData == odometry_data_) {
			odometryCallbackHandler();
		} else if(pData == sonar_data_) {
			sonarCallbackHandler();
		} else if(pData == bumper_data_) {
			bumperDataCallbackHandler();
		}
	}

private:
	// wrapping the type specific Robot.set/getFeatureTYPE methods with templates
	template<typename FeatureType>
	void setFeature(std::string name, FeatureType value);
	template<typename FeatureType>
	FeatureType getFeature(std::string name);

	void odometryCallbackHandler();
	void sonarCallbackHandler();
	void bumperDataCallbackHandler();

	void diagnosticsPublishingLoop(ros::Rate loop_rate);
	void checkSubscribersLoop(ros::Rate loop_rate);
	void dynamicReconfigureUpdaterLoop(
			dynamic_reconfigure::Server<metralabs_ros::ScitosG5Config> &dynamic_reconfigure_server,
			boost::recursive_mutex &mutex, ros::Rate loop_rate);

	void dynamicReconfigureCallback(metralabs_ros::ScitosG5Config& config, uint32_t level);

	void getFeatures(metralabs_ros::ScitosG5Config& config);


	void setVelocity(double translational_velocity, double rotational_velocity) {
		velocity_cmd_->writeLock();
		velocity_cmd_->setVelocity(translational_velocity, rotational_velocity);
		velocity_cmd_->writeUnlock(MTime::now());
		velocity_cmd_->setModified();
	}

	void resetBumper() {
		bumper_reset_cmd_->set(0, true, true, MetraLabs::base::MTime::now());
	}


	void driveCommandCallback(const geometry_msgs::TwistConstPtr& msg) {
		setVelocity(msg->linear.x, msg->angular.z);
		ROS_DEBUG("Received some speeds [%f %f]", msg->linear.x, msg->angular.z);
	}

	void bumperResetCallback(const std_msgs::EmptyConstPtr& dummy) {
		ROS_INFO("Resetting bumper");
		resetBumper();
	}


private:
	Application* app_;
	ClassFactory* class_factory_;
	Blackboard* blackboard_;
	Robot* robot_;

	BlackboardDataOdometry* odometry_data_;
	BlackboardDataRange* sonar_data_;
	BlackboardDataBatteryState* battery_state_data_;
	BlackboardDataBumper* bumper_data_;

	BlackboardDataVelocity* velocity_cmd_;
	BlackboardDataUInt8* bumper_reset_cmd_;


	ros::NodeHandle& node_handle_;
	tf::TransformBroadcaster tf_broadcaster_;

	ros::Publisher diagnostics_publisher_;
	ros::Publisher odom_publisher_;
	ros::Publisher sonar_publisher_;
	ros::Publisher bumper_publisher_;

	ros::Subscriber cmd_vel_subscriber_;
	ros::Subscriber bumper_reset_subscriber_;

	boost::recursive_mutex dynamic_reconfigure_mutex_;
	dynamic_reconfigure::Server<metralabs_ros::ScitosG5Config> dynamic_reconfigure_server_;

	bool sonar_is_requested_;
};

#endif
