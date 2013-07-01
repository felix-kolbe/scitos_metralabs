
#include <string>

#include <ros/ros.h>

#include "ScitosBase.h"

#define SCHUNK_NOT_AMTEC 0
#include "SchunkServer.h"



void robotArmThread(ScitosBase& scitos_base, ros::NodeHandle nh_schunk) {
	ROS_INFO("Starting ros schunk connector...");
	SchunkServer schunk_server(nh_schunk);

	try {
		while (nh_schunk.ok())
			boost::this_thread::sleep(boost::posix_time::minutes(42));
	}
	catch(const boost::thread_interrupted&) {
		ROS_INFO("Robot arm thread was interrupted and returns, stopping the arm interface.");
	}
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "metralabs_ros");

	ros::NodeHandle nh_private("~");
	ros::NodeHandle nh_scitos("scitos");
	ros::NodeHandle nh_schunk("schunk");


	/// read parameters

	bool disable_arm;
	nh_private.param("disable_arm", disable_arm, false);

	std::string scitos_config_file;
	nh_private.param<std::string>("scitos_config_file", scitos_config_file,
			"/opt/MetraLabs/MLRobotic/etc/config/SCITOS-G5_without_Head_config.xml");

	ros::Duration(0.9).sleep(); // wait to let the running me close its scitos connection


	/// initialize robot base & node components

	ROS_INFO("Starting robot base...");
	ScitosBase base(scitos_config_file.c_str(), argc, argv, nh_scitos);

	base.setFeature(FEATURE_SONAR, false);

	boost::thread robot_arm_thread_;
	if(nh_private.hasParam("robot_arm_class")) {
		robot_arm_thread_ = boost::thread(robotArmThread, boost::ref(base), boost::ref(nh_schunk));
	}


	/// start main loop

	ROS_INFO("Initializing done, starting loop");
	ros::spin();


	// clean up

	if(robot_arm_thread_.joinable()) {
		robot_arm_thread_.interrupt();
		robot_arm_thread_.join();
	}
	else
		ROS_WARN("robot arm thread not joinable, should this happen?");

	base.setFeature(FEATURE_SONAR, false);

	return 0;
}
