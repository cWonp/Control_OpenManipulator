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
#include "../include/covid19_communication_client/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace covid19_communication {
ros::Subscriber visual_control_sub;//from vision
ros::Publisher visual_control_pub;//to arm
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
void QNode::VisualControlSub(const controller_arm_msg::VisualControl::ConstPtr &msg)
{
    to_arm_msg = *msg;
    Q_EMIT VisualCallbackSignal();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"covid19_communication_client");
    //    if ( ! ros::master::check() ) {
    //        return false;
    //    }
    ros::start();
    ros::NodeHandle n;
    // Add your ros communications here.
    visual_control_sub = n.subscribe("VisualControl_TCP", 1000, &QNode::VisualControlSub, this);
    visual_control_pub = n.advertise<controller_arm_msg::VisualControl>("VisualControl", 1000);
    start();
    return true;
}



void QNode::run() {
    ros::Rate loop_rate(100);//100
    while ( ros::ok() ) {

        ros::spinOnce();
        loop_rate.sleep();
    }
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
}  // namespace covid19_communication
