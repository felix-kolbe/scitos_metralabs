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

	void publishOdometry(double x, double y, double theta, double v, double w);
	void getOdometry(double& x, double& y, double& theta, double& v, double& w);

	void publishSonar(std::vector<RangeData::Measurement> measurements);
	void getSonar(std::vector<RangeData::Measurement>& measurements);
	void publishSonarConfig(const RangeData::Config* sonar_config);
	void getSonarConfig(const RangeData::Config*& sonar_config);

	void publishBatteryState(float voltage, float current, int16_t charge_state,
			int16_t remaining_time, int16_t charger_status, ros::Time timestamp);
	void getBatteryState(float& voltage, float& current, int16_t& charge_state,
			int16_t& remaining_time, int16_t& charger_status, ros::Time& timestamp);

	void publishBumperState(bool bumper_pressed, bool motor_stop);
	void getBumperState(bool& bumper_pressed, bool& motor_stop);

	void setVelocity(double translational_velocity, double rotational_velocity);

	void resetBumper() {
		bumper_reset_cmd_->set(0, true, true, MetraLabs::base::MTime::now());
	}

	// wrapping the type specific Robot.set/getFeatureTYPE methods with templates
	template<typename FeatureType>
	void setFeature(std::string name, FeatureType value);
	template<typename FeatureType>
	FeatureType getFeature(std::string name);


	virtual void dataChanged(const BlackboardData* pData) {
		if(pData == odometry_data_) {
			odometryCallbackHandler();
		} else if(pData == sonar_data_) {
			sonarCallbackHandler();
		} else if(pData == battery_state_data_) {
			batteryStateCallbackHandler();
		} else if(pData == bumper_data_) {
			bumperDataCallbackHandler();
		}
	}

private:
	void odometryCallbackHandler() {
		MTime time;
		Pose pose;
		Velocity velocity;
		float mileage;

		odometry_data_->readLock();
		odometry_data_->getData(pose, velocity, mileage);
		time = odometry_data_->getTimeStamp();
		odometry_data_->readUnlock();

		publishOdometry(pose.getX(), pose.getY(), pose.getPhi(),
					velocity.getVelocityTranslational(),
					velocity.getVelocityRotational());

		/// The odometry position and velocities of the robot
		ros::Time odom_time = ros::Time().fromNSec(time.getTimeValue()*1000000);
		double x = pose.getX();
		double y = pose.getY();
		double th = pose.getPhi();
		double vx = velocity.getVelocityTranslational();
		double vth = velocity.getVelocityRotational();

		// since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		/// odometry tf
		geometry_msgs::TransformStamped odom_tf;
		odom_tf.header.stamp = odom_time;
		odom_tf.header.frame_id = "/odom";
		odom_tf.child_frame_id = "/base_link";

		odom_tf.transform.translation.x = x;
		odom_tf.transform.translation.y = y;
		odom_tf.transform.rotation = odom_quat;

		// publish the transform
		tf_broadcaster_.sendTransform(odom_tf);

		/// odometry data
		nav_msgs::Odometry odom_msg;
		odom_msg.header.stamp = odom_time;
		odom_msg.header.frame_id = "/odom";
		odom_msg.child_frame_id = "/base_link";

		// set the position
		odom_msg.pose.pose.position.x = x;
		odom_msg.pose.pose.position.y = y;
		odom_msg.pose.pose.orientation = odom_quat;

		// set the velocity
		odom_msg.twist.twist.linear.x = vx;
		odom_msg.twist.twist.angular.z = vth;

		// publish the message
		odom_publisher_.publish(odom_msg);
	}

	void sonarCallbackHandler() {
		// TODO optimize method: full method if-requested and one-time getConfig
		sonar_data_->readLock();
		const RangeData::Vector& sonar_data = sonar_data_->getRangeData();
		const RangeData::Config* sonar_config = sonar_data_->getConfig();
		MTime timestamp = odometry_data_->getTimeStamp();
		sonar_data_->readUnlock();

		const std::vector<RangeData::Measurement> measurements = sonar_data;
		ros::Time sonar_time = ros::Time().fromNSec(timestamp.getTimeValue()*1000000);

		publishSonar(measurements);
		publishSonarConfig(sonar_config);

		if(sonar_is_requested_) {
			static const RangeData::Config* sonar_config = NULL;

			// load config once
			if (sonar_config == NULL) {
				getSonarConfig(sonar_config);    // TODO what if nonzero rubbish is read?
//				std::cout << "sonar_config was NULL and now we read: " << sonar_config << std::endl;
			}
			// if config is loaded, proceed..
			if (sonar_config != NULL) {
				const RangeData::ConfigCircularArc* sonar_config_circular =
						dynamic_cast<const RangeData::ConfigCircularArc*>(sonar_config);

				/// sonar tf

				// calculate transforms (once)
				static std::vector<tf::Transform> sonar_transforms;
				static bool sonar_transforms_calculated = false;

				if(!sonar_transforms_calculated) {
					sonar_transforms_calculated = true;
					float angle = sonar_config_circular->first_sensor_angle;
					for (unsigned int i = 0; i < sonar_config_circular->sensor_cnt; ++i) {
						float x = sonar_config_circular->offset_from_center * std::cos(angle) -0.075;
						float y = sonar_config_circular->offset_from_center * std::sin(angle);
						tf::Quaternion quat;
						quat.setEuler(0, 0, angle);
						tf::Transform* transform = new tf::Transform(quat, tf::Vector3(x, y, 0.25));
						sonar_transforms.push_back(*transform);
						angle += sonar_config_circular->sensor_angle_dist;
					}
					// broadcast all transforms once
					std::vector<tf::Transform>::iterator it = sonar_transforms.begin();
					for (int i = 0; it != sonar_transforms.end(); ++it) {
						char targetframe[20];
						sprintf(targetframe, "/sonar/sonar_%02d", i++);
						tf_broadcaster_.sendTransform(
								tf::StampedTransform(*it, sonar_time, "/base_link", targetframe)
							);
					}
				}

				/// sonar data msg

				sensor_msgs::Range sonar_msg;
				sonar_msg.header.stamp = sonar_time;
				sonar_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
				sonar_msg.field_of_view = sonar_config_circular->cone_angle;  // DEG_TO_RAD(15);	// from manual
				sonar_msg.min_range = 0.2;	// from manual
				sonar_msg.max_range = 3;	// from manual

				// each time send only one sonar data and one transform to reduce repeating messages
				{
					static int next_sonar_to_send = 0;
					char next_sonar_frame[20];
					sprintf(next_sonar_frame, "/sonar/sonar_%02d", next_sonar_to_send);

					// range data
//					ROS_DEBUG("Sonar #%d: value: %1.3f status: %d", next_sonar_to_send, measurements.at(next_sonar_to_send).range, measurements.at(next_sonar_to_send).err.std);
					switch(measurements.at(next_sonar_to_send).err.std) {
						case RangeData::RANGE_OKAY:
						case RangeData::RANGE_OBJECT_SOMEWHERE_IN_RANGE:
						case RangeData::RANGE_PROBABLY_OKAY:
						case RangeData::RANGE_PROBABLY_OBJECT_SOMEWHERE_IN_RANGE:
							sonar_msg.range = measurements.at(next_sonar_to_send).range;
							break;
						case RangeData::RANGE_NO_OBJECT_WITHIN_RANGE:
						case RangeData::RANGE_PROBABLY_NO_OBJECT_WITHIN_RANGE:
						case RangeData::RANGE_INVALID_MEASUREMENT:
						case RangeData::RANGE_ERR_SENSOR_BROKEN:
						case RangeData::RANGE_MASKED:
						default:
							;// TODO maybe send a zero-range message to make the old value obsolete, if that isn't useful
							sonar_msg.range = 0;
					}
					sonar_msg.header.frame_id = next_sonar_frame;
					sonar_publisher_.publish(sonar_msg);

					// resend also one transform (the according, why not)
					tf_broadcaster_.sendTransform( tf::StampedTransform(
							sonar_transforms.at(next_sonar_to_send), ros::Time::now(), "/base_link", next_sonar_frame ) );

					++next_sonar_to_send %= measurements.size();
				}

	//		this is code to send all sonar measurements at one time
	//			std::vector<RangeData::Measurement>::iterator itM = measurements.begin();
	//			for(int i=0; itM != measurements.end(); ++itM) {
	////				std::cout<<itM->range<<" ";
	//
	//				char targetframe[20];
	//				sprintf(targetframe, "/sonar/sonar_%2d", i++);
	//				sonar.header.frame_id = targetframe;
	//				sonar.range = itM->range+1;
	//
	//				//publish the message
	//				sonar_publisher_.publish(sonar);
	//			}

			} // if sonar config loaded
		} // if sonar active
	}

	void batteryStateCallbackHandler() {
		battery_state_data_->readLock();
		publishBatteryState(
				battery_state_data_->getVoltage(),
				battery_state_data_->getCurrent(),
				battery_state_data_->getChargeState(),
				battery_state_data_->getRemainingTime(),
				battery_state_data_->getChargerStatus(),
				ros::Time().fromNSec(battery_state_data_->getTimeStamp().getTimeValue()*1000000)
				);
		battery_state_data_->readUnlock();
	}

	void bumperDataCallbackHandler() {
		bumper_data_->readLock();
		BumperData::Vector bumper_values = bumper_data_->getBumperData();
		MTime timestamp = odometry_data_->getTimeStamp();
		bumper_data_->readUnlock();

		bool bumper_pressed = false;
		bool motor_stop = false;

#define BUMPER_CODE_PUSHED 0x12
#define BUMPER_CODE_LOCKED 0x02

		for (BumperData::Vector::const_iterator it = bumper_values.begin(); it != bumper_values.end(); ++it) {
			if (*it == BUMPER_CODE_PUSHED) {
				bumper_pressed = true;
				motor_stop = true;
				break;  // no next bumper part would change any value
			}
			else if (*it == BUMPER_CODE_LOCKED) {
				motor_stop = true;
			}
		}

		publishBumperState(bumper_pressed, motor_stop);

		metralabs_ros::ScitosG5Bumper bumper_msg;
		bumper_msg.header.stamp = ros::Time().fromNSec(timestamp.getTimeValue()*1000000);
		bumper_msg.bumper_pressed = bumper_pressed;
		bumper_msg.motor_stop = motor_stop;

		bumper_publisher_.publish(bumper_msg);
	}


	void driveCommandCallback(const geometry_msgs::TwistConstPtr& msg) {
		setVelocity(msg->linear.x, msg->angular.z);
		ROS_DEBUG("Received some speeds [%f %f]", msg->linear.x, msg->angular.z);
	}

	void bumperResetCallback(const std_msgs::EmptyConstPtr& dummy) {
		ROS_INFO("Resetting bumper");
		resetBumper();
	}

	void dynamicReconfigureCallback(metralabs_ros::ScitosG5Config& config, uint32_t level);

	void getFeatures(metralabs_ros::ScitosG5Config& config);

	void checkSubscribersLoop(ros::Rate loop_rate) {
		// set state variable and hardware to same state
		sonar_is_requested_ = false;
		robot_->setFeatureBool(FEATURE_SONAR, sonar_is_requested_);
		while (node_handle_.ok()) {
			/// enable or disable sonar if someone or no one is listening
			bool sonar_had_been_requested = sonar_is_requested_;
			sonar_is_requested_ = sonar_publisher_.getNumSubscribers() != 0;
			// this check allows to override sonar state via dynamic reconfigure
			if(sonar_is_requested_ != sonar_had_been_requested) {
				robot_->setFeatureBool(FEATURE_SONAR, sonar_is_requested_);
				ROS_INFO_STREAM("Switching sonar feature to: " << (sonar_is_requested_?"true":"false"));
			}
			loop_rate.sleep();
		}
	}

	void diagnosticsPublishingLoop(ros::Rate loop_rate) {
		while (node_handle_.ok()) {
			float voltage;
			float current;
			int16_t charge_state;
			int16_t remaining_time;
			int16_t charger_status;
			ros::Time timestamp;
			getBatteryState(voltage, current, charge_state, remaining_time, charger_status, timestamp);

			diagnostic_msgs::DiagnosticStatus battery_status;
			battery_status.level = diagnostic_msgs::DiagnosticStatus::OK;
			battery_status.name = "Battery";
			battery_status.message = "undefined";
			battery_status.hardware_id = "0a4fcec0-27ef-497a-93ba-db39808ec1af";

// TODO do me parameters
#define 	VOLTAGE_ERROR_LEVEL	23		// and below
#define 	VOLTAGE_WARN_LEVEL	24		// and below
#define 	VOLTAGE_MID_LEVEL	26		// and below // above means HIGH_LEVEL
#define 	VOLTAGE_FULL_LEVEL	28.8	// and above
#define 	CHARGER_PLUGGED 	1

			if(voltage < VOLTAGE_ERROR_LEVEL && charger_status != CHARGER_PLUGGED)
				battery_status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
			else if(voltage < VOLTAGE_WARN_LEVEL && charger_status != CHARGER_PLUGGED)
				battery_status.level = diagnostic_msgs::DiagnosticStatus::WARN;

			// build text message
			battery_status.message = "High";
			if(voltage < VOLTAGE_MID_LEVEL)
				battery_status.message = "Mid";
			if(voltage < VOLTAGE_WARN_LEVEL)
				battery_status.message = "Low";
			if(voltage < VOLTAGE_ERROR_LEVEL)
				battery_status.message = "Depleted";
			battery_status.message += charger_status == CHARGER_PLUGGED ? ", charging" : ", discharging";
			if(voltage >= VOLTAGE_FULL_LEVEL)
				battery_status.message = "Fully charged";


			battery_status.values.resize(5);
			std::stringstream ss;
			battery_status.values[0].key = "Voltage";
			ss << voltage << " V";
			battery_status.values[0].value = ss.str();

			ss.str("");
			battery_status.values[1].key = "Current";
			ss << current << " A";
			battery_status.values[1].value = ss.str();

			ss.str("");
			battery_status.values[2].key = "ChargeState";
			ss << charge_state << " %";
			battery_status.values[2].value = ss.str();

			ss.str("");
			battery_status.values[3].key = "RemainingTime";
			ss << remaining_time << " min";
			battery_status.values[3].value = ss.str();

			battery_status.values[4].key = "ChargerStatus";
			battery_status.values[4].value = charger_status == CHARGER_PLUGGED ? "plugged" : "unplugged";

			/// combine and publish statii as array
			diagnostic_msgs::DiagnosticArray diag_array;
			diag_array.header.stamp = timestamp;
			diag_array.status.push_back(battery_status);
			diagnostics_publisher_.publish(diag_array);

			// This will adjust as needed per iteration
			loop_rate.sleep();
		}
	}

	void dynamicReconfigureUpdaterLoop(dynamic_reconfigure::Server<metralabs_ros::ScitosG5Config> &dynamic_reconfigure_server,
			boost::recursive_mutex &mutex, ros::Rate loop_rate) {
		metralabs_ros::ScitosG5Config config;
		while (node_handle_.ok()) {
			getFeatures(config);// update config to current hardware state
			boost::recursive_mutex::scoped_lock lock(mutex);
			dynamic_reconfigure_server.updateConfig(config);
			lock.unlock();
			loop_rate.sleep();
		}
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


	double odom_x_;
	double odom_y_;
	double odom_theta_;
	double odom_v_;
	double odom_w_;

	double command_v_;
	double command_w_;

	std::vector<RangeData::Measurement> range_measurements_;
	const RangeData::Config* sonar_config_;
	bool sonar_is_requested_;

	float battery_voltage_;
	float battery_current_;
	int16_t battery_charge_state_;
	int16_t battery_remaining_time_;
	int16_t battery_charger_status_;
	ros::Time battery_timestamp_;

	bool bumper_pressed_;
	bool motor_stop_;
};

#endif
