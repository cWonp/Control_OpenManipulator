#ifndef MAINCONTROLLER_HPP_
#define MAINCONTROLLER_HPP_

#include "ros/ros.h"
#include <iostream>
#include <fstream>

#include "trajectory_planner.hpp"
#include "kinematics_solver.hpp"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

//msg........................................................................
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include "controller_arm_msg/StateInfo.h"
#include "controller_arm_msg/GuiControl.h"
#include "controller_arm_msg/Monitor.h"
#include "controller_arm_msg/VisualControl.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"

using namespace std;

/*****************************************************************************
** ROS Topic Publiser
*****************************************************************************/
vector<ros::Publisher> JointAngle2Gazebo_pub;
ros::Publisher StateArmPub;
ros::Publisher MonitorPub;
/*****************************************************************************
** ROS Topic Subsriber
*****************************************************************************/
ros::Subscriber GuiControlSub;
ros::Subscriber VisualControlSub;

#define tick            0.01 //[s] maincontrol timer tick (10ms)
#define sec2tick        100
#define InitTime        7 //[s]

//Velocity limit
#define OriThreshold    2   //[deg]
#define PosThresholdX    0.2   //[mm]
#define PosThresholdY    0.01   //[mm]
#define PosThresholdZ    0.2   //[mm]

//Run mode
#define STOP                0
#define POS_CTRL            1
#define ANG_CTRL            2
#define MOTION_ROTATING     3
#define STEP_VelocityLimit  4

//Limit
#define RunTimeMin 100 // 1s


//GUI CMD
#define RUN             0
#define GetFK           1
#define ChangeInitPos   2
#define ChnageInitAng   3
#define LinearMotion    4
#define VisualCtrlPos   5
#define VisualCtrlAng   6

//InputOrientation
#define YPR         0
#define RCM         1
#define HeadingAng  2

//InputPosition
#define GlobalPos   0
#define LocalPos    1
#define Length      2


struct moveinfo
{
    int RunMode;
    int OriMode;
    double RunTime;

    Vector3d PosFrom;
    Vector3d PosTo;

    Matrix3d RotFrom;
    Vector3d OriEulFrom;
    Eigen::Quaterniond QuatFrom;
    Matrix3d RotTo;
    Vector3d OriEulTo;
    Eigen::Quaterniond QuatTo;
    Vector3d oriRCM;

    VectorNd JointAngFrom;
    VectorNd JointAngTo;
};
struct motion_rotate{
    double num;
    double ang;
    unsigned int time;
    unsigned int entire_time;
};
struct armstate
{
    Vector3d Pos;
    Vector3d OriEul;
    Matrix3d Rot;
    Eigen::Quaterniond Quat;

    VectorNd JointAng;
};

class MainController{
public:
    DynamixelWorkbench DXL;

    KinematicsSolver Solver;

    TrajectoryPlanner PatternX; TrajectoryPlanner PatternY; TrajectoryPlanner PatternZ;
    TrajectoryPlanner PatternOrientation;
    TrajectoryPlanner PatternAng[6];
    TrajectoryPlanner PatternRotateMotion;

    moveinfo Move;
    motion_rotate RotateMotion;
    armstate ArmState;

    void GuiControlSub(const controller_arm_msg::GuiControl::ConstPtr &msg); // Gui node subscribe CallBack
    void VisualControlSub(const controller_arm_msg::VisualControl::ConstPtr &msg);
    void TimerCallback(const ros::TimerEvent&); // Timer CallBack

    void execute(int RunMode);

    void init();
    bool DXLinit();
    bool ReadInitParam();

    VectorNd ReadDXLPos();
    void RunDynamixel(VectorNd Ang);

    void RunGazebo(VectorNd Ang);
    void BroadcastTF(Vector3d Pos, Vector3d Ori);

    bool Run_RealRobot = true;
    bool Run_Gazebo = true;
    bool UsingAngLimit = true;

    Vector3d PosPrevious = Move.PosFrom;
    Vector3d OriPrevious = Move.OriEulFrom;

    double PosThreshold[3];
private:
    controller_arm_msg::StateInfo StateArmMsg;
    controller_arm_msg::Monitor MonitorMsg;
    dynamixel_workbench_msgs::DynamixelCommand DynamixelSrv;

    unsigned int t = 0; // 10ms
};

MainController Controller;


#endif /* MAINCONTROLLER_HPP_ */
