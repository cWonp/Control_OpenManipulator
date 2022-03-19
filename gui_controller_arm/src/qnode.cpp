/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/gui_controller_arm/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gui_controller_arm {

ros::Publisher GuiControlPub;
ros::Subscriber StateArmInfoSub;

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

void QNode::StateInfoCallback(const controller_arm_msg::StateInfo::ConstPtr& msg){
    StateArmInfo = *msg;
    Q_EMIT StateInfoSignal();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"gui_controller_arm");
//	if ( ! ros::master::check() ) {
//		return false;
//	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

        // Add your ros communications here.
        GuiControlPub = n.advertise<controller_arm_msg::GuiControl>("GuiControl", 100);

        StateArmInfoSub = n.subscribe("StateInfo", 100, &QNode::StateInfoCallback, this);

	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);

	while ( ros::ok() ) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

}  // namespace gui_controller_arm
