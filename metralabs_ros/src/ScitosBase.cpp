#include "ScitosBase.h"


//	FEATURE_BOOL = 0,  /**< A boolean feature. */
//	FEATURE_INT,       /**< A integer feature. */
//	FEATURE_FLOAT,     /**< A float feature. */
//	FEATURE_STRING     /**< A string feature. */

template<>
void ScitosBase::setFeature<bool>(std::string name, bool value) {
	robot_->setFeatureBool(name, value);
}
template<>
void ScitosBase::setFeature<int>(std::string name, int value) {
	robot_->setFeatureInt(name, value);
}
template<>
void ScitosBase::setFeature<float>(std::string name, float value) {
	robot_->setFeatureFloat(name, value);
}
template<>
void ScitosBase::setFeature<std::string>(std::string name, std::string value) {
	robot_->setFeatureString(name, value);
}


template<>
bool ScitosBase::getFeature(std::string name) {
	bool value;
	robot_->getFeatureBool(name, value);
	return value;
}
template<>
int ScitosBase::getFeature(std::string name) {
	int value;
	robot_->getFeatureInt(name, value);
	return value;
}
template<>
float ScitosBase::getFeature(std::string name) {
	float value;
	robot_->getFeatureFloat(name, value);
	return value;
}
template<>
std::string ScitosBase::getFeature(std::string name) {
	MString value;
	robot_->getFeatureString(name, value);
	return value;
}



ScitosBase::ScitosBase(const char* config_file, int pArgc, char* pArgv[], ros::NodeHandle& nh) :
	node_handle_(nh),
	dynamic_reconfigure_server_(dynamic_reconfigure_mutex_),
	sonar_is_requested_(false)
{
	using namespace MetraLabs::base;
	using namespace MetraLabs::robotic::base;
	using namespace MetraLabs::robotic::robot;

	Error tErr;

	///////////////////////////////////////////////////////////////////////////
	// Initialization

	// Create a general application object. This will also initialize
	// the library MetraLabsBase with the command line arguments.
	app_ = new Application(pArgc, pArgv);
	if (app_ == NULL) {
		ROS_FATAL("Can't create the application!");
		exit(-1);
	}

	// Get the class factory from the application
	class_factory_ = app_->getClassFactory();
	if (class_factory_ == NULL) {
		ROS_FATAL("Cannot get the ClassFactory!");
		exit(-1);
	}

	// Load some parameters for the robot SCITOS-G5
	ParameterNode tRobotCfg("RobotCfg");
	if ((tErr = tRobotCfg.readFromFile(config_file)) != OK) {
		ROS_FATAL("Can't read parameter file. Code: %s", getErrorString(tErr).c_str());
		exit(-1);
	}

	///////////////////////////////////////////////////////////////////////////

	// Get the blackboard
	blackboard_ = app_->getBlackboard();
	if (blackboard_ == NULL) {
		ROS_FATAL("Cannot get the Blackboard!");
		exit(-1);
	}

	///////////////////////////////////////////////////////////////////////////
	// Robot creation

	// Create the robot interface for SCITOS-G5
	robot_ = createInstance<Robot>(class_factory_,
			"b07fb034-83c1-446c-b2df-0dd6aa46eef6");
	if (robot_ == NULL) {
		ROS_FATAL("Failed to create the robot.");
		exit(-1);
	}

	// Pre-Initialize the robot
	int tries = 3;
	while(tries-->0 && (tErr = robot_->preInitializeClient(&tRobotCfg)) != OK) {
		ROS_WARN("Failed to pre-initialize the robot. Waiting and retrying..");
		sleep(2);
	}
	if (tErr != OK) {
		ROS_FATAL("Failed to pre-initialize the robot. Code: %s", getErrorString(tErr).c_str());
		exit(-1);
	}

	// Assign robot to blackboard
	robot_->setPhysicalName("Robot",                  "MyRobot");
	robot_->setPhysicalName("BatteryState",           "MyRobot.BatteryState");
	robot_->setPhysicalName("Drive.Odometry",         "MyRobot.Odometry");
	robot_->setPhysicalName("Drive.VelocityCmd",      "MyRobot.VelocityCmd");
	robot_->setPhysicalName("RangeFinder.Sonar.Data", "MyRobot.Sonar");
	robot_->setPhysicalName("Bumper.Bumper.Data",     "MyRobot.Bumper");
	robot_->setPhysicalName("Bumper.Bumper.ResetCmd", "MyRobot.BumperResetCmd");
	if ((tErr = robot_->assignToBlackboard(blackboard_, true)) != OK) {
		ROS_FATAL("Failed to assign the robot to the blackboard. Code: %s", getErrorString(tErr).c_str());
		exit(-1);
	}

	// Initialize the robot
	if ((tErr = robot_->initializeClient(&tRobotCfg)) != OK) {
		ROS_FATAL("Failed to initialize the robot. Code: %s", getErrorString(tErr).c_str());
		exit(-1);
	}

	///////////////////////////////////////////////////////////////////////////
	// Blackboard data: register listeners

	// Odometry
	odometry_data_ = NULL;
	tErr = getDataFromBlackboard<BlackboardDataOdometry>(blackboard_,
			"MyRobot.Odometry", odometry_data_);
	if (tErr != OK) {
		ROS_FATAL("Failed to get the odometry data from the blackboard. Code: %s", getErrorString(tErr).c_str());
		exit(-1);
	}
	odometry_data_->addCallback(this);

	// Sonar
	sonar_data_ = NULL;
	tErr = getDataFromBlackboard<BlackboardDataRange>(blackboard_,
			"MyRobot.Sonar", sonar_data_);
	if (tErr != OK) {
		ROS_FATAL("Failed to get the Sonar data from the blackboard. Is it specified in the XML config? Code: %s", getErrorString(tErr).c_str());
//		exit(-1);  no, let the robot start wihtout sonar, even if it wasn't specified in the XML config.
	} else {
		sonar_data_->addCallback(this);
	}

	// Battery
	battery_state_data_ = NULL;
	tErr = getDataFromBlackboard<BlackboardDataBatteryState>(blackboard_,
			"MyRobot.BatteryState", battery_state_data_);
	if (tErr != OK) {
		ROS_FATAL("Failed to get the battery state data from the blackboard. Code: %s", getErrorString(tErr).c_str());
		exit(-1);
	}
	// battery_state_data_->addCallback(this);  // replaced with diagnostics thread fetching on demand

	// Bumper
	bumper_data_ = NULL;
	tErr = getDataFromBlackboard<BlackboardDataBumper>(blackboard_,
			"MyRobot.Bumper", bumper_data_);
	if (tErr != OK) {
		ROS_FATAL("Failed to get the bumper data from the blackboard. Code: %s", getErrorString(tErr).c_str());
		exit(-1);
	}
	bumper_data_->addCallback(this);

	///////////////////////////////////////////////////////////////////////////
	// Blackboard data: register publishers

	// Bumper reset command
	bumper_reset_cmd_ = NULL;
	tErr = getDataFromBlackboard<BlackboardDataUInt8>(blackboard_,
			"MyRobot.BumperResetCmd", bumper_reset_cmd_);
	if (tErr != OK) {
		ROS_FATAL("Failed to get the bumper reset command from the blackboard. Code: %s", getErrorString(tErr).c_str());
		exit(-1);
	}

	// Velocity command
	velocity_cmd_ = NULL;
	tErr = getDataFromBlackboard<BlackboardDataVelocity>(blackboard_,
			"MyRobot.VelocityCmd", velocity_cmd_);
	if (tErr != OK) {
		ROS_FATAL("Failed to get the velocity command from the blackboard. Code: %s", getErrorString(tErr).c_str());
		exit(-1);
	}


	///////////////////////////////////////////////////////////////////////////
	// Activate ROS interface

	odom_publisher_ = node_handle_.advertise<nav_msgs::Odometry>("/odom", 20);
	sonar_publisher_ = node_handle_.advertise<sensor_msgs::Range>("/sonar", 50);
	bumper_publisher_ = node_handle_.advertise<metralabs_msgs::ScitosG5Bumper>("/bumper", 20);

	cmd_vel_subscriber_ = node_handle_.subscribe("/cmd_vel", 1, &ScitosBase::driveCommandCallback, this);
	bumper_reset_subscriber_ = node_handle_.subscribe("/bumper_reset", 1, &ScitosBase::bumperResetCallback, this);

	// init automatic sonar de/activatior
	boost::thread(&ScitosBase::checkSubscribersLoop, this, ros::Rate(1));


	///////////////////////////////////////////////////////////////////////////
	// Activation

	// Start the blackboard
	if ((tErr = blackboard_->startBlackboard()) != OK) {
		ROS_FATAL("Failed to start the blackboard. Code: %s", getErrorString(tErr).c_str());
		exit(-1);
	}

	///////////////////////////////////////////////////////////////////////////
	// Start the robot
	if ((tErr = robot_->startClient()) != OK) {
		ROS_FATAL("Failed to start the robot system. Code: %s", getErrorString(tErr).c_str());
		exit(-1);
	}


	/// intialize diagnostics

	ROS_INFO("Starting diagnostics...");

	diagnostics_publisher_ = node_handle_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 50);
	boost::thread(&ScitosBase::diagnosticsPublishingLoop, this, ros::Rate(2));


	/// intialize dynamic_reconfigure

	ROS_INFO("Starting dynamic_reconfigure...");

	// update config to current hardware state
	metralabs_ros::ScitosG5Config config;
	getFeatures(config);	// The first time the read config is totally wrong, maybe it's the previous state from rosparam
	ROS_INFO_STREAM("This is what the first read gives... " << config.EBC1_Enable24V);
//	getFeatures(config);	// It's no timing issue as far as I tested it. 		TODO fix or proof as stable..

	// mutex needed for dynreconf.updateConfig, see non existent manual, eh I mean source
	boost::recursive_mutex::scoped_lock dyn_reconf_lock(dynamic_reconfigure_mutex_);
//	ROS_INFO_STREAM("Updating with current config from hardware... " << config.EBC1_Enable24V);
	dynamic_reconfigure_server_.updateConfig(config);
	dyn_reconf_lock.unlock();

	// init reconfigure publisher
	boost::thread(&ScitosBase::dynamicReconfigureUpdaterLoop, this, boost::ref(dynamic_reconfigure_server_),
			boost::ref(dynamic_reconfigure_mutex_), ros::Rate(2));

	// init reconfigure callback
	dynamic_reconfigure::Server<metralabs_ros::ScitosG5Config>::CallbackType dyn_reconf_callback_ptr;
	dyn_reconf_callback_ptr = boost::bind(&ScitosBase::dynamicReconfigureCallback, this, _1, _2);
	dynamic_reconfigure_server_.setCallback(dyn_reconf_callback_ptr);

}

ScitosBase::~ScitosBase() {
	setFeature(FEATURE_SONAR, false);

	Error tErr;
	if ((tErr = blackboard_->stopBlackboard()) != OK)
		ROS_ERROR("Failed to stop the blackboard. Code: %s", getErrorString(tErr).c_str());

	// Stop the robot.
	if ((tErr = robot_->stopClient()) != OK)
		ROS_ERROR("Failed to stop the robot system. Code: %s", getErrorString(tErr).c_str());

	// Destroy the robot
	if ((tErr = robot_->destroyClient()) != OK)
		ROS_ERROR("Failed to destroy the robot system. Code: %s", getErrorString(tErr).c_str());

	// Delete the application object
	delete app_;
}


void ScitosBase::dynamicReconfigureCallback(metralabs_ros::ScitosG5Config& config, uint32_t level) {
	// I wrote this macro because I couldn't find a way to read the configs parameters generically,
	// and with this macro the actual feature name only has to be named once. Improvements welcome.
	#define MAKRO_SET_FEATURE(NAME)	\
		ROS_INFO("Setting feature %s to %s", #NAME, config.NAME?"True":"False"); \
		setFeature(#NAME, config.NAME)

	MAKRO_SET_FEATURE(EBC0_Enable5V);
	MAKRO_SET_FEATURE(EBC0_Enable12V);
	MAKRO_SET_FEATURE(EBC0_Enable24V);
	MAKRO_SET_FEATURE(EBC1_Enable5V);
	MAKRO_SET_FEATURE(EBC1_Enable12V);
	MAKRO_SET_FEATURE(EBC1_Enable24V);
	MAKRO_SET_FEATURE(FreeRunMode);
	MAKRO_SET_FEATURE(SonarsActive);
	MAKRO_SET_FEATURE(StatusDisplayKnobLock);
	MAKRO_SET_FEATURE(StatusDisplayLED);

	ROS_DEBUG("Now reading again: (why is this rubbish?)");	// TODO fix me
	metralabs_ros::ScitosG5Config config_read;
	getFeatures(config_read);
}

void ScitosBase::getFeatures(metralabs_ros::ScitosG5Config& config) {
	// I wrote this macro because I couldn't find a way to read the configs parameters generically,
	// and with this macro the actual feature name only has to be named once. Improvements welcome.
	#define MAKRO_GET_FEATURE(NAME)	\
		ROS_DEBUG("Current hardware feature %s is %s", #NAME, config.NAME?"True":"False"); \
		config.NAME = getFeature<typeof(config.NAME)>(std::string(#NAME))

	MAKRO_GET_FEATURE(EBC0_Enable5V);
	MAKRO_GET_FEATURE(EBC0_Enable12V);
	MAKRO_GET_FEATURE(EBC0_Enable24V);
	MAKRO_GET_FEATURE(EBC1_Enable5V);
	MAKRO_GET_FEATURE(EBC1_Enable12V);
	MAKRO_GET_FEATURE(EBC1_Enable24V);
	MAKRO_GET_FEATURE(FreeRunMode);
	MAKRO_GET_FEATURE(SonarsActive);
	MAKRO_GET_FEATURE(StatusDisplayKnobLock);
	MAKRO_GET_FEATURE(StatusDisplayLED);
}


void ScitosBase::odometryCallbackHandler() {
	MTime time;
	Pose pose;
	Velocity velocity;
	float mileage;

	odometry_data_->readLock();
	odometry_data_->getData(pose, velocity, mileage);
	time = odometry_data_->getTimeStamp();
	odometry_data_->readUnlock();

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

void ScitosBase::sonarCallbackHandler() {
	static const RangeData::Config* sonar_config = NULL;

	sonar_data_->readLock();
	MTime timestamp = odometry_data_->getTimeStamp();
	const RangeData::Vector& sonar_data = sonar_data_->getRangeData();
	if (sonar_config == NULL)
		sonar_config = sonar_data_->getConfig();
	sonar_data_->readUnlock();

	const std::vector<RangeData::Measurement> measurements = sonar_data;
	ros::Time sonar_time = ros::Time().fromNSec(timestamp.getTimeValue()*1000000);


	if(!sonar_is_requested_) {
		setFeature(FEATURE_SONAR, false);
	} else {
		if (sonar_config == NULL) {
			ROS_WARN_THROTTLE(0.5, "Could not yet read sonar config.");
		} else {
			// if config is loaded, proceed..
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
			//	ROS_DEBUG("Sonar #%d: value: %1.3f status: %d", next_sonar_to_send, measurements.at(next_sonar_to_send).range, measurements.at(next_sonar_to_send).err.std);
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

void ScitosBase::bumperDataCallbackHandler() {
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
		ROS_DEBUG("BumperData was: %X bumper_pressed: %d motor_stop: %d", *it, bumper_pressed, motor_stop);
	}

	metralabs_msgs::ScitosG5Bumper bumper_msg;
	bumper_msg.header.stamp = ros::Time().fromNSec(timestamp.getTimeValue()*1000000);
	bumper_msg.bumper_pressed = bumper_pressed;
	bumper_msg.motor_stop = motor_stop;

	bumper_publisher_.publish(bumper_msg);
}

void ScitosBase::diagnosticsPublishingLoop(ros::Rate loop_rate) {
	// send diagnostics message for battery state but with lower frequency
	// robot itself publishes with 10 Hz into Blackboard
	while (node_handle_.ok()) {
		battery_state_data_->readLock();
		MString name = static_cast<BatteryState*>(battery_state_data_)->getName();
		float voltage = battery_state_data_->getVoltage();
		float current = battery_state_data_->getCurrent();
		int16_t charge_state = battery_state_data_->getChargeState();
		int16_t remaining_time = battery_state_data_->getRemainingTime();
		int16_t charger_status= battery_state_data_->getChargerStatus();
		MTime timestamp = battery_state_data_->getTimeStamp();
		battery_state_data_->readUnlock();

		diagnostic_msgs::DiagnosticStatus battery_status;
		battery_status.name = "Scitos G5: Battery";
		battery_status.message = "undefined";
		battery_status.hardware_id = name;

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
		else
			battery_status.level = diagnostic_msgs::DiagnosticStatus::OK;

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

		// set values
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
		diag_array.header.stamp = ros::Time().fromNSec(timestamp.getTimeValue()*1000000);
		diag_array.status.push_back(battery_status);
		diagnostics_publisher_.publish(diag_array);

		// This will adjust as needed per iteration
		loop_rate.sleep();
	}
}

void ScitosBase::checkSubscribersLoop(ros::Rate loop_rate) {
	// set state variable and hardware to same state
	sonar_is_requested_ = false;
	setFeature(FEATURE_SONAR, false);
	while (node_handle_.ok()) {
		/// enable or disable sonar if someone or no one is listening
		bool sonar_had_been_requested = sonar_is_requested_;
		sonar_is_requested_ = sonar_publisher_.getNumSubscribers() != 0;
		// this check allows to override sonar state via dynamic reconfigure and avoids overhead
		if(sonar_is_requested_ != sonar_had_been_requested) {
			setFeature(FEATURE_SONAR, sonar_is_requested_);
			ROS_INFO_STREAM("Switching sonar feature to: " << (sonar_is_requested_ ? "enabled" : "disabled"));
		}
		loop_rate.sleep();
	}
}

void ScitosBase::dynamicReconfigureUpdaterLoop(
		dynamic_reconfigure::Server<metralabs_ros::ScitosG5Config> &dynamic_reconfigure_server,
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
