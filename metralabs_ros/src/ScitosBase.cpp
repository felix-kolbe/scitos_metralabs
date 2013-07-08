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
		fprintf(stderr, "FATAL: Can't create the application!\n");
		exit(-1);
	}

	// Get the class factory from the application
	class_factory_ = app_->getClassFactory();
	if (class_factory_ == NULL) {
		fprintf(stderr, "FATAL: Cannot get the ClassFactory!\n");
		exit(-1);
	}

	// Load some parameters for the robot SCITOS-G5
	ParameterNode tRobotCfg("RobotCfg");
	if ((tErr = tRobotCfg.readFromFile(config_file)) != OK) {
		fprintf(stderr, "FATAL: Can't read parameter file. Code: %s\n", getErrorString(tErr).c_str());
		exit(-1);
	}

	///////////////////////////////////////////////////////////////////////////

	// Get the blackboard
	blackboard_ = app_->getBlackboard();
	if (blackboard_ == NULL) {
		fprintf(stderr, "FATAL: Cannot get the Blackboard!\n");
		exit(-1);
	}

	///////////////////////////////////////////////////////////////////////////
	// Robot creation

	// Create the robot interface for SCITOS-G5
	robot_ = createInstance<Robot>(class_factory_,
			"b07fb034-83c1-446c-b2df-0dd6aa46eef6");
	if (robot_ == NULL) {
		fprintf(stderr, "FATAL: Failed to create the robot. Abort!\n");
		exit(-1);
	}

	// Pre-Initialize the robot
	int tries = 3;
	while(tries-->0 && (tErr = robot_->preInitializeClient(&tRobotCfg)) != OK) {
		ROS_WARN("Failed to pre-initialize the robot. Code: %s. Waiting and retrying..", getErrorString(tErr).c_str());
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
		fprintf(stderr, "FATAL: Failed to assign the robot to the blackboard. Code: %s\n", getErrorString(tErr).c_str());
		exit(-1);
	}

	// Initialize the robot
	if ((tErr = robot_->initializeClient(&tRobotCfg)) != OK) {
		fprintf(stderr, "FATAL: Failed to initialize the robot. Code: %s\n", getErrorString(tErr).c_str());
		exit(-1);
	}

	///////////////////////////////////////////////////////////////////////////
	// Blackboard data: register listeners

	// Odometry
	odometry_data_ = NULL;
	tErr = getDataFromBlackboard<BlackboardDataOdometry>(blackboard_,
			"MyRobot.Odometry", odometry_data_);
	if (tErr != OK) {
		fprintf(stderr, "FATAL: Failed to get the odometry data from the blackboard! Code: %s\n", getErrorString(tErr).c_str());
		exit(-1);
	}
	odometry_data_->addCallback(this);

	// Sonar
	sonar_data_ = NULL;
	tErr = getDataFromBlackboard<BlackboardDataRange>(blackboard_,
			"MyRobot.Sonar", sonar_data_);
	if (tErr != OK) {
		fprintf(stderr, "FATAL: Failed to get the Sonar data from the blackboard! Is it specified in the XML config? Code: %s\n", getErrorString(tErr).c_str());
//		exit(-1);  no, let the robot start wihtout sonar, even if it wasn't specified in the XML config.
	} else {
		sonar_data_->addCallback(this);
	}

	// Battery
	battery_state_data_ = NULL;
	tErr = getDataFromBlackboard<BlackboardDataBatteryState>(blackboard_,
			"MyRobot.BatteryState", battery_state_data_);
	if (tErr != OK) {
		fprintf(stderr, "FATAL: Failed to get the battery state data from the blackboard! Code: %s\n", getErrorString(tErr).c_str());
		exit(-1);
	}
	// battery_state_data_->addCallback(this);  // replaced with diagnostics thread fetching on demand

	// Bumper
	bumper_data_ = NULL;
	tErr = getDataFromBlackboard<BlackboardDataBumper>(blackboard_,
			"MyRobot.Bumper", bumper_data_);
	if (tErr != OK) {
		fprintf(stderr, "FATAL: Failed to get the bumper data from the blackboard! Code: %s\n", getErrorString(tErr).c_str());
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
		fprintf(stderr, "FATAL: Failed to get the bumper reset command from the blackboard! Code: %s\n", getErrorString(tErr).c_str());
		exit(-1);
	}

	// Velocity command
	velocity_cmd_ = NULL;
	tErr = getDataFromBlackboard<BlackboardDataVelocity>(blackboard_,
			"MyRobot.VelocityCmd", velocity_cmd_);
	if (tErr != OK) {
		fprintf(stderr, "FATAL: Failed to get the velocity command from the blackboard! Code: %s\n", getErrorString(tErr).c_str());
		exit(-1);
	}


	///////////////////////////////////////////////////////////////////////////
	// Activate ROS interface

	odom_publisher_ = node_handle_.advertise<nav_msgs::Odometry>("/odom", 20);
	sonar_publisher_ = node_handle_.advertise<sensor_msgs::Range>("/sonar", 50);
	bumper_publisher_ = node_handle_.advertise<metralabs_ros::ScitosG5Bumper>("/bumper", 20);

	cmd_vel_subscriber_ = node_handle_.subscribe("/cmd_vel", 1, &ScitosBase::driveCommandCallback, this);
	bumper_reset_subscriber_ = node_handle_.subscribe("/bumper_reset", 1, &ScitosBase::bumperResetCallback, this);

	// init automatic sonar de/activatior
	boost::thread(&ScitosBase::checkSubscribersLoop, this, ros::Rate(1));


	///////////////////////////////////////////////////////////////////////////
	// Activation

	// Start the blackboard
	if ((tErr = blackboard_->startBlackboard()) != OK) {
		fprintf(stderr, "FATAL: Failed to start the blackboard. Code: %s\n", getErrorString(tErr).c_str());
		exit(-1);
	}

	///////////////////////////////////////////////////////////////////////////
	// Start the robot
	if ((tErr = robot_->startClient()) != OK) {
		fprintf(stderr, "FATAL: Failed to start the robot system. Code: %s\n", getErrorString(tErr).c_str());
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


void ScitosBase::setVelocity(double translational_velocity, double rotational_velocity) {
	velocity_cmd_->writeLock();
	velocity_cmd_->setVelocity(translational_velocity, rotational_velocity);
	velocity_cmd_->writeUnlock(MTime::now());
	velocity_cmd_->setModified();
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


ScitosBase::~ScitosBase() {
	Error tErr;
	if ((tErr = blackboard_->stopBlackboard()) != OK)
		fprintf(stderr, "ERROR: Failed to stop the blackboard. Code: %s\n", getErrorString(tErr).c_str());

	// Stop the robot.
	if ((tErr = robot_->stopClient()) != OK)
		fprintf(stderr, "ERROR: Failed to stop the robot system. Code: %s\n", getErrorString(tErr).c_str());

	// Destroy the robot
	if ((tErr = robot_->destroyClient()) != OK)
		fprintf(stderr, "ERROR: Failed to destroy the robot system. Code: %s\n", getErrorString(tErr).c_str());

	// Delete the application object
	delete app_;
}
