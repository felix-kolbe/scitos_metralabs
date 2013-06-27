
#include <string>

#include <ros/ros.h>

#include "ScitosBase.h"

#define SCHUNK_NOT_AMTEC 0
#include "SchunkServer.h"


#define FEATURE_LASER	( "EBC0_Enable24V" )
#define FEATURE_ARM		( "EBC1_Enable24V" )


int main(int argc, char **argv)
{
	ros::init(argc, argv, "metralabs_ros");

	ros::NodeHandle nh_private("~");
	ros::NodeHandle nh_scitos("scitos");
	ros::NodeHandle nh_schunk("schunk");


	/// read parameters

	bool disable_arm;
	nh_private.param("disable_arm", disable_arm, false);

	std::string scitos_config_file;
	nh_private.param<string>("scitos_config_file", scitos_config_file,
			"/opt/MetraLabs/MLRobotic/etc/config/SCITOS-G5_without_Head_config.xml");

	ros::Duration(0.9).sleep(); // wait to let the running me close its scitos connection


	/// initialize robot base & node components

	ROS_INFO("Starting robot base...");
	ScitosBase base(scitos_config_file.c_str(), argc, argv, nh_scitos);

	base.setFeature(FEATURE_SONAR, false);

	if(!disable_arm) {
		ros::Duration(0.5).sleep(); // let ScitosBase connect to robot
		base.setFeature(FEATURE_ARM, true);
	}


	/// intialize robot arm

	ROS_INFO("Starting ros schunk connector...");
	SchunkServer schunkServer(nh_schunk);


	/// start main loop

	ROS_INFO("Initializing done, starting loop");

	ros::spin();

	base.setFeature(FEATURE_ARM, false);
	base.setFeature(FEATURE_SONAR, false);

	return 0;
}

